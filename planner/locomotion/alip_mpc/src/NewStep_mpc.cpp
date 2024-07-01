#include "NewStep_mpc.hpp"
#include "util/util.hpp"

using namespace std;


// Constructor
NewStep_mpc::NewStep_mpc(string & horizon, string & intervals, const bool &new_solver)
{
    //util::PrettyConstructor(2, "Alip Mpc");

    new_solver_ = new_solver;
    // Casadi function definitions
    N_steps_ = stoi(horizon); //C: ver que significa stoi
    //N_xsol_ = n_xlip_ * N_steps_;
    //N_ufpsol_ = n_ufp_ * N_steps_;
    if (new_solver){
        solver_LS_ = "new_draco3_LS_simplempc_Ns" + horizon + "_Ni" + intervals + "_qrqp";
        f_solver_LS_ = casadi::external(solver_LS_);

        solver_RS_ = "new_draco3_RS_simplempc_Ns" + horizon + "_Ni" + intervals + "_qrqp";
        f_solver_RS_ = casadi::external(solver_RS_);
    }
    else{
        solver_LS_ = "draco_LS_simplempc_Ns" + horizon + "_Ni" + intervals + "_qrqp";
        f_solver_LS_ = casadi::external(solver_LS_);

        solver_RS_ = "draco_RS_simplempc_Ns" + horizon + "_Ni" + intervals + "_qrqp";
        f_solver_RS_ = casadi::external(solver_RS_);
    }   
    // cout << "--> Casadi Solvers Generated\n";

    // Initialize terrain deques


    //Ver que hacer con esto; Doing weird things with kx

    /*
    N_steps_ = stoi(horizon);
    for (int i = 0; i < N_steps_; i++)
    {   
        cout << "kx_init_ :  " << kx_init_ << endl;
        kx_traj_.push_back(kx_init_);
        ky_traj_.push_back(ky_init_);
        mux_traj_.push_back(mu_init_);
        muy_traj_.push_back(mu_init_);
    }
    */
   //this values will be used if not changed by SetParameters
    mass_ = 39.15342;
    ufp_x_max = 0.6;
    ufp_y_max_ = 0.4;
    ufp_y_min_ = 0.1;
}

// Destructor
//C: ver que significa destructor
NewStep_mpc::~NewStep_mpc() {}

// Update
void NewStep_mpc::Update_(const input_data_t &input_data,
                                   output_data_t &output_data, full_horizon_sol &full_sol)
{
    // Extract inputs



    x_lip_current_.assign(input_data.xlip_current, input_data.xlip_current + 4);
    stance_leg_ = input_data.stance_leg; // -1 -> left stance next currently in right
    zH_ = input_data.zH;
    Ts_ = input_data.Ts;
    Tr_ = input_data.Tr;
    leg_width_ = input_data.leg_width;
    Lx_offset_ = input_data.Lx_offset;
    Ly_des_ = input_data.Ly_des;
    kx_new_ = input_data.kx;
    ky_new_ = input_data.ky;
    mu_new_ = input_data.mu;


    if (mu_new_ < 0.05)
    {
        mu_new_ = mux_traj_.back(); // incase udp cuts out just use previous mu
    }

    // Set Opt parameters: p_x_init, p_stanceLeg, p_m, p_zH, p_Ts, p_Tr, p_leg_width, p_Lx_offset, p_Ly_des, p_ufp_max,p_ufp_min, p_k, p_mu, p_Q_term
    // vector<double> xlip_init = {xc_this_, yc_this_, Lx_this_, Ly_this_};   // x_init is 4 x 1

    // C: haber que hacer con estos limites, los puedo poner en hpp?
    vector<double> ufp_max;
    vector<double> ufp_min;
    double mech_limit;
    if (new_solver_){
        mech_limit = ufp_x_max / 2;
        ufp_max = {new_ufp_x_max_ / 2, new_ufp_y_max_};  // Max distance of step length x
        ufp_min = {-new_ufp_x_max_ / 2, ufp_y_min_}; // Max distance of step length y
    }
    else {
        ufp_max = {ufp_x_max / 2, ufp_y_max_};  // Max distance of step length x
        ufp_min = {-ufp_x_max / 2, ufp_y_min_}; // Max distance of step length y
    }

    // Update Terrain traj parameters
    kx_traj_.assign(N_steps_, kx_new_);
    ky_traj_.assign(N_steps_, ky_new_);
    mux_traj_.assign(N_steps_, mu_new_);
    muy_traj_.assign(N_steps_, 0.6);

    /* Convert terrain vectors to casadi matrix objects */
    casadi::Matrix<double> k_traj(n_ufp_, N_steps_);
    casadi::Matrix<double> mu_traj(n_ufp_, N_steps_);
    for (int i = 0; i < n_ufp_; i++)
    {
        for (int j = 0; j < N_steps_; j++)
        {
            if (i == 0) // x-component
            {
                k_traj(i, j) = kx_traj_[j];
                mu_traj(i, j) = mux_traj_[j];
            }
            else // y-component
            {
                k_traj(i, j) = ky_traj_[j];
                mu_traj(i, j) = muy_traj_[j];
            }
        }
    }

    /* Terminal cost parameter */
    casadi::DM Q_term = casadi::DM::eye(n_xlip_);
    //Q_term *= 50;
    //Q_term(3,3) = 50;
    Q_term(0,0) = q_term_0_;
    Q_term(1,1) = q_term_1_;
    Q_term(2,2) = q_term_2_;
    Q_term(3,3) = q_term_3_; 

    /* MPC Solution */
    double ufp_x_sol, ufp_y_sol;
    double xfp, yfp; //Lx, Ly;


    casadi::Matrix<double> xlip_guess(n_xlip_, N_steps_);
    casadi::Matrix<double> ufp_guess(n_ufp_, N_steps_);
    // Initial guess for warm start
    if (iter_ > 0)
    {
        casadi::Matrix<double> xlip_guess = reshape(casadi::Matrix<double>(xlip_guess_), n_xlip_, N_steps_);
        casadi::Matrix<double> ufp_guess = reshape(casadi::Matrix<double>(ufp_guess_), n_ufp_, N_steps_);
    }

    vector<casadi::DM> arg;
    if (new_solver_){
        casadi::Matrix<double> cas_x_com_init_step = casadi::Matrix<double>(x_com_init_step_);

        // Solve optimization problem
        arg = {xlip_guess, ufp_guess, x_lip_current_, stance_leg_, mass_, zH_, Ts_, Tr_, leg_width_, Lx_offset_, Ly_des_, ufp_max, ufp_min, k_traj, mu_traj, Q_term, cas_x_com_init_step, mech_limit};
    }
    else{
        // Solve optimization problem
        arg = {xlip_guess, ufp_guess, x_lip_current_, stance_leg_, mass_, zH_, Ts_, Tr_, leg_width_, Lx_offset_, Ly_des_, ufp_max, ufp_min, k_traj, mu_traj, Q_term, y_mech_max_};
    }

    vector<casadi::DM> result_solver;
    //Different solver for each leg
    if (stance_leg_ == -1) result_solver = f_solver_LS_(arg); //left_stance, means i want to calculate right stance next.
    else result_solver = f_solver_RS_(arg);   //i want to calculate left stance. First result will be left

    vector<double> xlip_sol = (vector<double>)result_solver.at(0);
    vector<double> ufp_sol = (vector<double>)result_solver.at(1);
    vector<double> ufp_wrt_com_sol = (vector<double>)result_solver.at(2);

    //cout << "xlip_sol" << xlip_sol.size() << endl << xlip_sol;
    //cout << "ufp sol " << ufp_sol.size() << endl << ufp_sol;

    ufp_x_sol = ufp_sol[0];
    ufp_y_sol = ufp_sol[1];

    // Safety check for bad solutions
    // median of ufpxsol and limits


    vector<double> ufpx_check{ufp_x_sol, ufp_max[0], ufp_min[0]};
    auto m = ufpx_check.begin() + ufpx_check.size() / 2;
    nth_element(ufpx_check.begin(), m, ufpx_check.end());
    ufp_x_sol = ufpx_check[ufpx_check.size() / 2];
    // median of ufpysol and limits
   if (new_solver_){
    if (stance_leg_ == -1){
            vector<double> ufpy_check{x_com_init_step_[1]-0.2, ufp_y_sol, -ufp_min[1]};
            auto l = ufpy_check.begin() + ufpy_check.size() /2;
            nth_element(ufpy_check.begin(), l, ufpy_check.end());
            ufp_y_sol = ufpy_check[ufpy_check.size() / 2];
    }
    else{
            vector<double> ufpy_check{ufp_min[1], ufp_y_sol, x_com_init_step_[1]+0.2};
            // cout << "median vector: " << ufpy_check << endl;
            auto l = ufpy_check.begin() + ufpy_check.size() / 2;
            nth_element(ufpy_check.begin(), l, ufpy_check.end());
            ufp_y_sol = ufpy_check[ufpy_check.size() / 2];
        }
    }
    else{
        if (stance_leg_ == -1) // left stance next left swing
        {
            vector<double> ufpy_check{-ufp_max[1], ufp_y_sol, -ufp_min[1]};
            // cout << "median vector: " << ufpy_check << endl;
            auto l = ufpy_check.begin() + ufpy_check.size() / 2;
            nth_element(ufpy_check.begin(), l, ufpy_check.end());
            ufp_y_sol = ufpy_check[ufpy_check.size() / 2];
        }
        else // right stance
        {
            vector<double> ufpy_check{ufp_min[1], ufp_y_sol, ufp_max[1]};
            // cout << "median vector: " << ufpy_check << endl;
            auto l = ufpy_check.begin() + ufpy_check.size() / 2;
            nth_element(ufpy_check.begin(), l, ufpy_check.end());
            ufp_y_sol = ufpy_check[ufpy_check.size() / 2];
        }
    }


    /* Foot placement relative to current COM */
    xfp = xlip_sol[0];
    yfp = xlip_sol[1];

    /* Store solution for warm start */ 
    xlip_guess_ = xlip_sol;
    ufp_guess_ = ufp_sol;

    /* Increase solver loop iteration */
    iter_++;
    //cout << "x_sol " << ufp_x_sol << endl;
    //cout << "y_sol " << ufp_y_sol << endl;

    // Update output data struct
    output_data.ufp_wrt_st[0] = ufp_x_sol;
    output_data.ufp_wrt_st[1] = ufp_y_sol;
    output_data.ufp_wrt_com[0] = xfp;
    output_data.ufp_wrt_com[1] = yfp;

    //new output data full solver solution
    full_sol.xlip_sol = xlip_sol;
    full_sol.ufp_sol = ufp_sol;

    return;
}


void NewStep_mpc::SetParameters(const YAML::Node &node) {
  try {
    util::ReadParameter(node, "total_mass", mass_);
    util::ReadParameter(node, "ufp_x_max", ufp_x_max);
    util::ReadParameter(node, "ufp_y_max", ufp_y_max_);
    util::ReadParameter(node, "ufp_y_min", ufp_y_min_);
    util::ReadParameter(node, "new_ufp_x_max", new_ufp_x_max_);
    util::ReadParameter(node, "new_ufp_y_max", new_ufp_y_max_);
    util::ReadParameter(node, "q_term_0", q_term_0_);
    util::ReadParameter(node, "q_term_1", q_term_1_);
    util::ReadParameter(node, "q_term_2", q_term_2_);
    util::ReadParameter(node, "q_term_3", q_term_3_);
    util::ReadParameter(node, "y_mech_max", y_mech_max_);


  } catch (const std::runtime_error &e) {
    std::cerr << "Error reading parameter [" << e.what() << "] at file: ["
              << __FILE__ << "]" << std::endl;
    std::exit(EXIT_FAILURE);
  }
}




