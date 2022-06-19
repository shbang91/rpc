#pragma once

class Planner {
public:
  Planner() {}
  virtual ~Planner() = default;

  virtual void Initialize() = 0;
  virtual void GetSolution() = 0;

protected:
};
