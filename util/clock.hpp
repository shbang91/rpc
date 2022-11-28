#include <chrono>

using namespace std::chrono;

class Clock {
public:
  Clock() {}
  ~Clock() = default;

  void Start() { start_time_ = high_resolution_clock::now(); }

  void Stop() {
    end_time_ = high_resolution_clock::now();
    duration_ = duration_cast<microseconds>(end_time_ - start_time_);
  }

  double duration() const { return double(duration_.count()) * 1e-3; }

private:
  microseconds duration_;
  high_resolution_clock::time_point start_time_, end_time_;
};
