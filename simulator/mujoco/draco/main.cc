// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>

#include "mujoco/mujoco.h"
#include "simulate/array_safety.h"
#include "simulate/glfw_adapter.h"
#include "simulate/simulate.h"

// rpc related headers
#include "configuration.hpp"
#include "controller/draco_controller/draco_interface.hpp"

#define MUJOCO_PLUGIN_DIR "mujoco_plugin"

extern "C" {
#if defined(_WIN32) || defined(__CYGWIN__)
#include <windows.h>
#else
#if defined(__APPLE__)
#include <mach-o/dyld.h>
#endif
#include <sys/errno.h>
#include <unistd.h>
#endif
}

namespace {
namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

// constants
const double syncMisalign =
    0.1; // maximum mis-alignment before re-sync (simulation seconds)
const double simRefreshFraction =
    0.7;                       // fraction of refresh available for simulation
const int kErrorLength = 1024; // load error string length

// model and data
mjModel *m = nullptr;
mjData *d = nullptr;

// control noise variables
mjtNum *ctrlnoise = nullptr;

// rpc controller
Interface *draco_interface = nullptr;
DracoSensorData *draco_sensor_data = nullptr;
DracoCommand *draco_command = nullptr;

// mujoco joint & actuator maps
std::unordered_map<std::string, int> mj_jnt_map_;
std::unordered_map<std::string, int> mj_act_map_;

using Seconds = std::chrono::duration<double>;

//---------------------------------------- plugin handling
//-----------------------------------------

// return the path to the directory containing the current executable
// used to determine the location of auto-loaded plugin libraries
std::string getExecutableDir() {
#if defined(_WIN32) || defined(__CYGWIN__)
  constexpr char kPathSep = '\\';
  std::string realpath = [&]() -> std::string {
    std::unique_ptr<char[]> realpath(nullptr);
    DWORD buf_size = 128;
    bool success = false;
    while (!success) {
      realpath.reset(new (std::nothrow) char[buf_size]);
      if (!realpath) {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }

      DWORD written = GetModuleFileNameA(nullptr, realpath.get(), buf_size);
      if (written < buf_size) {
        success = true;
      } else if (written == buf_size) {
        // realpath is too small, grow and retry
        buf_size *= 2;
      } else {
        std::cerr << "failed to retrieve executable path: " << GetLastError()
                  << "\n";
        return "";
      }
    }
    return realpath.get();
  }();
#else
  constexpr char kPathSep = '/';
#if defined(__APPLE__)
  std::unique_ptr<char[]> buf(nullptr);
  {
    std::uint32_t buf_size = 0;
    _NSGetExecutablePath(nullptr, &buf_size);
    buf.reset(new char[buf_size]);
    if (!buf) {
      std::cerr << "cannot allocate memory to store executable path\n";
      return "";
    }
    if (_NSGetExecutablePath(buf.get(), &buf_size)) {
      std::cerr << "unexpected error from _NSGetExecutablePath\n";
    }
  }
  const char *path = buf.get();
#else
  const char *path = "/proc/self/exe";
#endif
  std::string realpath = [&]() -> std::string {
    std::unique_ptr<char[]> realpath(nullptr);
    std::uint32_t buf_size = 128;
    bool success = false;
    while (!success) {
      realpath.reset(new (std::nothrow) char[buf_size]);
      if (!realpath) {
        std::cerr << "cannot allocate memory to store executable path\n";
        return "";
      }

      std::size_t written = readlink(path, realpath.get(), buf_size);
      if (written < buf_size) {
        realpath.get()[written] = '\0';
        success = true;
      } else if (written == -1) {
        if (errno == EINVAL) {
          // path is already not a symlink, just use it
          return path;
        }

        std::cerr << "error while resolving executable path: "
                  << strerror(errno) << '\n';
        return "";
      } else {
        // realpath is too small, grow and retry
        buf_size *= 2;
      }
    }
    return realpath.get();
  }();
#endif

  if (realpath.empty()) {
    return "";
  }

  for (std::size_t i = realpath.size() - 1; i > 0; --i) {
    if (realpath.c_str()[i] == kPathSep) {
      return realpath.substr(0, i);
    }
  }

  // don't scan through the entire file system's root
  return "";
}

// scan for libraries in the plugin directory to load additional plugins
void scanPluginLibraries() {
  // check and print plugins that are linked directly into the executable
  int nplugin = mjp_pluginCount();
  if (nplugin) {
    std::printf("Built-in plugins:\n");
    for (int i = 0; i < nplugin; ++i) {
      std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
    }
  }

  // define platform-specific strings
#if defined(_WIN32) || defined(__CYGWIN__)
  const std::string sep = "\\";
#else
  const std::string sep = "/";
#endif

  // try to open the ${EXECDIR}/MUJOCO_PLUGIN_DIR directory
  // ${EXECDIR} is the directory containing the simulate binary itself
  // MUJOCO_PLUGIN_DIR is the MUJOCO_PLUGIN_DIR preprocessor macro
  const std::string executable_dir = getExecutableDir();
  if (executable_dir.empty()) {
    return;
  }

  const std::string plugin_dir = getExecutableDir() + sep + MUJOCO_PLUGIN_DIR;
  mj_loadAllPluginLibraries(
      plugin_dir.c_str(), +[](const char *filename, int first, int count) {
        std::printf("Plugins registered by library '%s':\n", filename);
        for (int i = first; i < first + count; ++i) {
          std::printf("    %s\n", mjp_getPluginAtSlot(i)->name);
        }
      });
}

//------------------------------------------- simulation
//-------------------------------------------

mjModel *LoadModel(const char *file, mj::Simulate &sim) {
  // this copy is needed so that the mju::strlen call below compiles
  char filename[mj::Simulate::kMaxFilenameLength];
  mju::strcpy_arr(filename, file);

  // make sure filename is not empty
  if (!filename[0]) {
    return nullptr;
  }

  // load and compile
  char loadError[kErrorLength] = "";
  mjModel *mnew = 0;
  if (mju::strlen_arr(filename) > 4 &&
      !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                    mju::sizeof_arr(filename) - mju::strlen_arr(filename) +
                        4)) {
    mnew = mj_loadModel(filename, nullptr);
    if (!mnew) {
      mju::strcpy_arr(loadError, "could not load binary model");
    }
  } else {
    mnew = mj_loadXML(filename, nullptr, loadError, kErrorLength);
    // remove trailing newline character from loadError
    if (loadError[0]) {
      int error_length = mju::strlen_arr(loadError);
      if (loadError[error_length - 1] == '\n') {
        loadError[error_length - 1] = '\0';
      }
    }
  }

  mju::strcpy_arr(sim.load_error, loadError);

  if (!mnew) {
    std::printf("%s\n", loadError);
    return nullptr;
  }

  // compiler warning: print and pause
  if (loadError[0]) {
    // mj_forward() below will print the warning message
    std::printf("Model compiled, but simulation warning (paused):\n  %s\n",
                loadError);
    sim.run = 0;
  }

  return mnew;
}

// TODO:
void SetJointAndActuatorMaps(
    mjModel *m, std::unordered_map<std::string, int> &joint_map,
    std::unordered_map<std::string, int> &actuator_map) {}

// TODO:
void SetInitialConfig(mjData *d) {
  d->qpos[2] = 0.95;
  // map (joint name -> joint idx)
  // d->qpos[9] = M_PI / 6;   // l_shoulder_aa
  // d->qpos[11] = -M_PI / 2; // l_elbow_fe
  // d->qpos[15] = -M_PI / 6; // r_shouler_aa
  // d->qpos[17] = -M_PI / 2; // r_elbow_fe

  // left leg
  // d->qpos[22] = -M_PI / 4;
  // d->qpos[23] = M_PI / 4;
  // d->qpos[24] = M_PI / 4;
  // d->qpos[25] = -M_PI / 4;

  // right leg
  // d->qpos[29] = -M_PI / 4;
  // d->qpos[30] = M_PI / 4;
  // d->qpos[31] = M_PI / 4;
  // d->qpos[32] = -M_PI / 4;
}

// TODO:
void MyController(const mjModel *m, mjData *d) {
  // if (m->nu == m->nv)
  // mju_scl(d->ctrl, d->qvel, -0.1, m->nv);
}

bool CopySensorData() {
  //=================================================
  // joint positions
  //=================================================
  // left leg
  draco_sensor_data->joint_pos_[0] = d->qpos[20];
  draco_sensor_data->joint_pos_[1] = d->qpos[21];
  draco_sensor_data->joint_pos_[2] = d->qpos[22];
  draco_sensor_data->joint_pos_[3] = d->qpos[23];
  draco_sensor_data->joint_pos_[4] = d->qpos[24];
  draco_sensor_data->joint_pos_[5] = d->qpos[25];
  draco_sensor_data->joint_pos_[6] = d->qpos[26];
  // left arm
  draco_sensor_data->joint_pos_[7] = d->qpos[8];
  draco_sensor_data->joint_pos_[8] = d->qpos[9];
  draco_sensor_data->joint_pos_[9] = d->qpos[10];
  draco_sensor_data->joint_pos_[10] = d->qpos[11];
  draco_sensor_data->joint_pos_[11] = d->qpos[12];
  draco_sensor_data->joint_pos_[12] = d->qpos[13];
  // neck
  draco_sensor_data->joint_pos_[13] = d->qpos[7];
  // right leg
  draco_sensor_data->joint_pos_[14] = d->qpos[27];
  draco_sensor_data->joint_pos_[15] = d->qpos[28];
  draco_sensor_data->joint_pos_[16] = d->qpos[29];
  draco_sensor_data->joint_pos_[17] = d->qpos[30];
  draco_sensor_data->joint_pos_[18] = d->qpos[31];
  draco_sensor_data->joint_pos_[19] = d->qpos[32];
  draco_sensor_data->joint_pos_[20] = d->qpos[33];
  // right arm
  draco_sensor_data->joint_pos_[21] = d->qpos[14];
  draco_sensor_data->joint_pos_[22] = d->qpos[15];
  draco_sensor_data->joint_pos_[23] = d->qpos[16];
  draco_sensor_data->joint_pos_[24] = d->qpos[17];
  draco_sensor_data->joint_pos_[25] = d->qpos[18];
  draco_sensor_data->joint_pos_[26] = d->qpos[19];

  //=================================================
  // joint velocities
  //=================================================
  // left leg
  draco_sensor_data->joint_vel_[0] = d->qvel[19];
  draco_sensor_data->joint_vel_[1] = d->qvel[20];
  draco_sensor_data->joint_vel_[2] = d->qvel[21];
  draco_sensor_data->joint_vel_[3] = d->qvel[22];
  draco_sensor_data->joint_vel_[4] = d->qvel[23];
  draco_sensor_data->joint_vel_[5] = d->qvel[24];
  draco_sensor_data->joint_vel_[6] = d->qvel[25];
  // left arm
  draco_sensor_data->joint_vel_[7] = d->qvel[7];
  draco_sensor_data->joint_vel_[8] = d->qvel[8];
  draco_sensor_data->joint_vel_[9] = d->qvel[9];
  draco_sensor_data->joint_vel_[10] = d->qvel[10];
  draco_sensor_data->joint_vel_[11] = d->qvel[11];
  draco_sensor_data->joint_vel_[12] = d->qvel[12];
  // neck
  draco_sensor_data->joint_vel_[13] = d->qvel[6];
  // right leg
  draco_sensor_data->joint_vel_[14] = d->qvel[26];
  draco_sensor_data->joint_vel_[15] = d->qvel[27];
  draco_sensor_data->joint_vel_[16] = d->qvel[28];
  draco_sensor_data->joint_vel_[17] = d->qvel[29];
  draco_sensor_data->joint_vel_[18] = d->qvel[30];
  draco_sensor_data->joint_vel_[19] = d->qvel[31];
  draco_sensor_data->joint_vel_[20] = d->qvel[32];
  // right arm
  draco_sensor_data->joint_vel_[21] = d->qvel[13];
  draco_sensor_data->joint_vel_[22] = d->qvel[14];
  draco_sensor_data->joint_vel_[23] = d->qvel[15];
  draco_sensor_data->joint_vel_[24] = d->qvel[16];
  draco_sensor_data->joint_vel_[25] = d->qvel[17];
  draco_sensor_data->joint_vel_[26] = d->qvel[18];

  //==============================================
  // floating base states for ground truth state estimation
  //==============================================
  draco_sensor_data->base_joint_pos_[0] = d->qpos[0];
  draco_sensor_data->base_joint_pos_[1] = d->qpos[1];
  draco_sensor_data->base_joint_pos_[2] = d->qpos[2];

  draco_sensor_data->base_joint_quat_[0] = d->qpos[4]; // q.x
  draco_sensor_data->base_joint_quat_[1] = d->qpos[5]; // q.y
  draco_sensor_data->base_joint_quat_[2] = d->qpos[6]; // q.z
  draco_sensor_data->base_joint_quat_[3] = d->qpos[3]; // q.w

  draco_sensor_data->base_joint_lin_vel_[0] = d->qvel[0];
  draco_sensor_data->base_joint_lin_vel_[1] = d->qvel[1];
  draco_sensor_data->base_joint_lin_vel_[2] = d->qvel[2];

  draco_sensor_data->base_joint_ang_vel_[0] = d->qvel[3];
  draco_sensor_data->base_joint_ang_vel_[1] = d->qvel[4];
  draco_sensor_data->base_joint_ang_vel_[2] = d->qvel[5];

  //==============================================
  // TODO:contact states (1. contact normal force, 2. swing foot height)
  //==============================================

  return true;
}

void CopyCommand() {
  // left arm
  d->ctrl[0] = draco_command->joint_trq_cmd_[7] +
               100 * (draco_command->joint_pos_cmd_[7] - d->qpos[8]) +
               5 * (draco_command->joint_vel_cmd_[7] - d->qvel[7]);
  d->ctrl[1] = draco_command->joint_trq_cmd_[8] +
               100 * (draco_command->joint_pos_cmd_[8] - d->qpos[9]) +
               5 * (draco_command->joint_vel_cmd_[8] - d->qvel[8]);
  d->ctrl[2] = draco_command->joint_trq_cmd_[9] +
               100 * (draco_command->joint_pos_cmd_[9] - d->qpos[10]) +
               5 * (draco_command->joint_vel_cmd_[9] - d->qvel[9]);
  d->ctrl[3] = draco_command->joint_trq_cmd_[10] +
               100 * (draco_command->joint_pos_cmd_[10] - d->qpos[11]) +
               5 * (draco_command->joint_vel_cmd_[10] - d->qvel[10]);
  d->ctrl[4] = draco_command->joint_trq_cmd_[11] +
               100 * (draco_command->joint_pos_cmd_[11] - d->qpos[12]) +
               5 * (draco_command->joint_vel_cmd_[11] - d->qvel[11]);
  d->ctrl[5] = draco_command->joint_trq_cmd_[12] +
               100 * (draco_command->joint_pos_cmd_[12] - d->qpos[13]) +
               5 * (draco_command->joint_vel_cmd_[12] - d->qvel[12]);
  // right arm
  d->ctrl[6] = draco_command->joint_trq_cmd_[21] +
               100 * (draco_command->joint_pos_cmd_[21] - d->qpos[14]) +
               5 * (draco_command->joint_vel_cmd_[21] - d->qvel[13]);
  d->ctrl[7] = draco_command->joint_trq_cmd_[22] +
               100 * (draco_command->joint_pos_cmd_[22] - d->qpos[15]) +
               5 * (draco_command->joint_vel_cmd_[22] - d->qvel[14]);
  d->ctrl[8] = draco_command->joint_trq_cmd_[23] +
               100 * (draco_command->joint_pos_cmd_[23] - d->qpos[16]) +
               5 * (draco_command->joint_vel_cmd_[23] - d->qvel[15]);
  d->ctrl[9] = draco_command->joint_trq_cmd_[24] +
               100 * (draco_command->joint_pos_cmd_[24] - d->qpos[17]) +
               5 * (draco_command->joint_vel_cmd_[24] - d->qvel[16]);
  d->ctrl[10] = draco_command->joint_trq_cmd_[25] +
                100 * (draco_command->joint_pos_cmd_[25] - d->qpos[18]) +
                5 * (draco_command->joint_vel_cmd_[25] - d->qvel[17]);
  d->ctrl[11] = draco_command->joint_trq_cmd_[26] +
                100 * (draco_command->joint_pos_cmd_[26] - d->qpos[19]) +
                5 * (draco_command->joint_vel_cmd_[26] - d->qvel[18]);
  // neck
  d->ctrl[12] = draco_command->joint_trq_cmd_[13] +
                100 * (draco_command->joint_pos_cmd_[13] - d->qpos[7]) +
                5 * (draco_command->joint_vel_cmd_[13] - d->qvel[6]);
  // left leg
  d->ctrl[13] = draco_command->joint_trq_cmd_[0] +
                400 * (draco_command->joint_pos_cmd_[0] - d->qpos[20]) +
                20 * (draco_command->joint_vel_cmd_[0] - d->qvel[19]);
  d->ctrl[14] = draco_command->joint_trq_cmd_[1] +
                500 * (draco_command->joint_pos_cmd_[1] - d->qpos[21]) +
                20 * (draco_command->joint_vel_cmd_[1] - d->qvel[20]);
  d->ctrl[15] = draco_command->joint_trq_cmd_[2] +
                500 * (draco_command->joint_pos_cmd_[2] - d->qpos[22]) +
                20 * (draco_command->joint_vel_cmd_[2] - d->qvel[21]);
  d->ctrl[16] = draco_command->joint_trq_cmd_[4] +
                300 * (draco_command->joint_pos_cmd_[4] - d->qpos[24]) +
                10 * (draco_command->joint_vel_cmd_[4] - d->qvel[23]);
  d->ctrl[17] = draco_command->joint_trq_cmd_[5] +
                70 * (draco_command->joint_pos_cmd_[5] - d->qpos[25]) +
                4 * (draco_command->joint_vel_cmd_[5] - d->qvel[24]);
  d->ctrl[18] = draco_command->joint_trq_cmd_[6] +
                70 * (draco_command->joint_pos_cmd_[6] - d->qpos[26]) +
                4 * (draco_command->joint_vel_cmd_[6] - d->qvel[25]);
  // right leg
  d->ctrl[19] = draco_command->joint_trq_cmd_[14] +
                400 * (draco_command->joint_pos_cmd_[14] - d->qpos[27]) +
                20 * (draco_command->joint_vel_cmd_[14] - d->qvel[26]);
  d->ctrl[20] = draco_command->joint_trq_cmd_[15] +
                500 * (draco_command->joint_pos_cmd_[15] - d->qpos[28]) +
                20 * (draco_command->joint_vel_cmd_[15] - d->qvel[27]);
  d->ctrl[21] = draco_command->joint_trq_cmd_[16] +
                500 * (draco_command->joint_pos_cmd_[16] - d->qpos[29]) +
                20 * (draco_command->joint_vel_cmd_[16] - d->qvel[28]);
  d->ctrl[22] = draco_command->joint_trq_cmd_[18] +
                300 * (draco_command->joint_pos_cmd_[18] - d->qpos[31]) +
                10 * (draco_command->joint_vel_cmd_[18] - d->qvel[30]);
  d->ctrl[23] = draco_command->joint_trq_cmd_[19] +
                70 * (draco_command->joint_pos_cmd_[19] - d->qpos[32]) +
                4 * (draco_command->joint_vel_cmd_[19] - d->qvel[31]);
  d->ctrl[24] = draco_command->joint_trq_cmd_[20] +
                70 * (draco_command->joint_pos_cmd_[20] - d->qpos[33]) +
                4 * (draco_command->joint_vel_cmd_[20] - d->qvel[32]);
  // std::cout << "----------------------------------------" << std::endl;
  // for (int i = 0; i < 25; ++i) {
  // std::cout << d->ctrl[i] << ", " << '\n';
  //}
}

// simulate in background thread (while rendering in main thread)
void PhysicsLoop(mj::Simulate &sim) {
  // cpu-sim syncronization point
  std::chrono::time_point<mj::Simulate::Clock> syncCPU;
  mjtNum syncSim = 0;

  int iter{0};
  //***************************************************
  // run until asked to exit (main simulation while loop)
  //***************************************************
  while (!sim.exitrequest.load()) {
    if (sim.droploadrequest.load()) {
      sim.LoadMessage(sim.dropfilename);
      mjModel *mnew = LoadModel(sim.dropfilename, sim);
      sim.droploadrequest.store(false);

      std::cout << "sim drop request!" << '\n';

      mjData *dnew = nullptr;
      if (mnew)
        dnew = mj_makeData(mnew);
      if (dnew) {
        sim.Load(mnew, dnew, sim.dropfilename);

        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        mj_deleteData(d);
        mj_deleteModel(m);

        m = mnew;
        d = dnew;
        mj_forward(m, d);

        // allocate ctrlnoise
        // free(ctrlnoise);
        // ctrlnoise = (mjtNum *)malloc(sizeof(mjtNum) * m->nu);
        // mju_zero(ctrlnoise, m->nu);
      } else {
        sim.LoadMessageClear();
      }
    }

    if (sim.uiloadrequest.load()) {
      sim.uiloadrequest.fetch_sub(1);
      sim.LoadMessage(sim.filename);
      mjModel *mnew = LoadModel(sim.filename, sim);
      mjData *dnew = nullptr;

      std::cout << "sim uiload request!" << '\n';

      if (mnew)
        dnew = mj_makeData(mnew);
      if (dnew) {
        sim.Load(mnew, dnew, sim.filename);

        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        mj_deleteData(d);
        mj_deleteModel(m);

        m = mnew;
        d = dnew;
        mj_forward(m, d);

        // allocate ctrlnoise
        // free(ctrlnoise);
        // ctrlnoise = static_cast<mjtNum *>(malloc(sizeof(mjtNum) * m->nu));
        // mju_zero(ctrlnoise, m->nu);
      } else {
        sim.LoadMessageClear();
      }
    }

    // sleep for 1 ms or yield, to let main thread run
    //  yield results in busy wait - which has better timing but kills battery
    //  life
    if (sim.run && sim.busywait) {
      std::this_thread::yield();
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    {
      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

      //***********************************************************
      // run only if model is present (main simulation loop)
      //***********************************************************
      if (m) {
        // running
        if (sim.run) {
          //*****************************************************
          if (CopySensorData())
            draco_interface->GetCommand(draco_sensor_data, draco_command);
          CopyCommand();
          // if (iter == 5)
          // exit(0);
          //*****************************************************

          bool stepped = false;

          // record cpu time at start of iteration
          const auto startCPU = mj::Simulate::Clock::now();

          // elapsed CPU and simulation time since last sync
          const auto elapsedCPU = startCPU - syncCPU;
          double elapsedSim = d->time - syncSim;

          // inject noise
          // if (sim.ctrl_noise_std) {
          // convert rate and scale to discrete time (Ornstein–Uhlenbeck)
          // mjtNum rate = mju_exp(-m->opt.timestep /
          // mju_max(sim.ctrl_noise_rate, mjMINVAL));
          // mjtNum scale = sim.ctrl_noise_std * mju_sqrt(1 - rate * rate);

          // for (int i = 0; i < m->nu; i++) {
          // update noise
          // ctrlnoise[i] =
          // rate * ctrlnoise[i] + scale * mju_standardNormal(nullptr);

          // apply noise
          // d->ctrl[i] = ctrlnoise[i];
          //}
          //}

          // requested slow-down factor
          double slowdown = 100 / sim.percentRealTime[sim.real_time_index];

          // misalignment condition: distance from target sim time is bigger
          // than syncmisalign
          bool misaligned = mju_abs(Seconds(elapsedCPU).count() / slowdown -
                                    elapsedSim) > syncMisalign;

          // out-of-sync (for any reason): reset sync times, step
          if (elapsedSim < 0 || elapsedCPU.count() < 0 ||
              syncCPU.time_since_epoch().count() == 0 || misaligned ||
              sim.speed_changed) {
            // re-sync
            syncCPU = startCPU;
            syncSim = d->time;
            sim.speed_changed = false;

            // run single step, let next iteration deal with timing
            mj_step(m, d);
            stepped = true;
          }

          // in-sync: step until ahead of cpu
          else {
            bool measured = false;
            mjtNum prevSim = d->time;

            double refreshTime = simRefreshFraction / sim.refresh_rate;

            // step while sim lags behind cpu and within refreshTime
            while (Seconds((d->time - syncSim) * slowdown) <
                       mj::Simulate::Clock::now() - syncCPU &&
                   mj::Simulate::Clock::now() - startCPU <
                       Seconds(refreshTime)) {
              // measure slowdown before first step
              if (!measured && elapsedSim) {
                sim.measured_slowdown =
                    std::chrono::duration<double>(elapsedCPU).count() /
                    elapsedSim;
                measured = true;
              }

              // call mj_step
              mj_step(m, d);
              stepped = true;

              // break if reset
              if (d->time < prevSim) {
                break;
              }
            }
          }

          // save current state to history buffer
          if (stepped) {
            sim.AddToHistory();
          }
        }

        // paused
        else {
          // run mj_forward, to update rendering and joint sliders
          mj_forward(m, d);
          sim.speed_changed = true;
        }
      }
    } // release std::lock_guard<std::mutex>
    iter++;
  }
}
} // namespace

//-------------------------------------- physics_thread
//--------------------------------------------

void PhysicsThread(mj::Simulate *sim, const char *filename) {
  // request loadmodel if file given (otherwise drag-and-drop)
  if (filename != nullptr) {
    sim->LoadMessage(filename);
    m = LoadModel(filename, *sim);
    if (m) {
      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

      // TEST print model
      // mj_printModel(m, "draco_mjcf_info");

      d = mj_makeData(m);
    }
    if (d) {
      // Construct controller
      //**********************************************
      draco_interface = new DracoInterface();
      draco_sensor_data = new DracoSensorData();
      draco_command = new DracoCommand();
      //**********************************************

      // Pass keystroke interrupt
      //**********************************************
      sim->Load(m, d, filename, draco_interface->interrupt_handler_);
      //**********************************************

      // Set up mujoco joint & actuator maps
      // ********************************************
      SetJointAndActuatorMaps(m, mj_jnt_map_, mj_act_map_);
      // ********************************************

      // Set initial configuration
      //**********************************************
      SetInitialConfig(d);
      //**********************************************

      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

      mj_forward(m, d);

      // allocate ctrlnoise
      // free(ctrlnoise);
      // ctrlnoise = static_cast<mjtNum *>(malloc(sizeof(mjtNum) * m->nu));
      // mju_zero(ctrlnoise, m->nu);
    } else {
      sim->LoadMessageClear();
    }
  }

  PhysicsLoop(*sim);

  // delete everything we allocated
  delete draco_interface;
  delete draco_sensor_data;
  delete draco_command;
  // free(ctrlnoise);
  mj_deleteData(d);
  mj_deleteModel(m);
}

//------------------------------------------ main
//--------------------------------------------------

// machinery for replacing command line error by a macOS dialog box when running
// under Rosetta
#if defined(__APPLE__) && defined(__AVX__)
extern void DisplayErrorDialogBox(const char *title, const char *msg);
static const char *rosetta_error_msg = nullptr;
__attribute__((used, visibility("default"))) extern "C" void
_mj_rosettaError(const char *msg) {
  rosetta_error_msg = msg;
}
#endif

// run event loop
int main(int argc, char **argv) {

  // display an error if running on macOS under Rosetta 2
#if defined(__APPLE__) && defined(__AVX__)
  if (rosetta_error_msg) {
    DisplayErrorDialogBox("Rosetta 2 is not supported", rosetta_error_msg);
    std::exit(1);
  }
#endif

  // print version, check compatibility
  std::printf("MuJoCo version %s\n", mj_versionString());
  if (mjVERSION_HEADER != mj_version()) {
    mju_error("Headers and library have different versions");
  }

  // scan for libraries in the plugin directory to load additional plugins
  scanPluginLibraries();

  mjvCamera cam;
  mjv_defaultCamera(&cam);

  mjvOption opt;
  mjv_defaultOption(&opt);

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  // simulate object encapsulates the UI
  auto sim =
      std::make_unique<mj::Simulate>(std::make_unique<mj::GlfwAdapter>(), &cam,
                                     &opt, &pert, /* is_passive = */ false);

  const char *filename = nullptr;

  // TODO: remove if statement
  if (argc > 1) {
    filename = argv[1];
  } else {
    filename = THIS_COM "robot_model/draco/draco_scene.xml";
  }

  // start physics thread
  std::thread physicsthreadhandle(&PhysicsThread, sim.get(), filename);

  // start simulation UI loop (blocking call)
  sim->RenderLoop();
  physicsthreadhandle.join();

  return 0;
}
