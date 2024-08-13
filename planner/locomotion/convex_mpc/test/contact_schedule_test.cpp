#include <vector>

#include <gtest/gtest.h>

#include "convex_mpc/contact_schedule.hpp"

namespace convexmpc {

class ContactScheduleTest : public ::testing::Test {
protected:
  virtual void SetUp() {
    T = 0.5;
    N = 15;
    dt = T / N;
  }

  virtual void TearDown() {}

  double T, dt;
  int N;
};

TEST_F(ContactScheduleTest, testContactSchedule) {
  ContactSchedule contact_schedule(T, N);
  EXPECT_DOUBLE_EQ(contact_schedule.T(), T);
  EXPECT_DOUBLE_EQ(contact_schedule.dt(), dt);
  EXPECT_EQ(contact_schedule.N(), N);
  for (int i = 0; i < N + 1; ++i) {
    EXPECT_EQ(contact_schedule.phase(i), 0);
  }
  EXPECT_EQ(contact_schedule.numActiveContacts(0), 4);
  for (int j = 0; j < 4; ++j) {
    EXPECT_TRUE(contact_schedule.isContactActive(0)[j]);
  }

  const double t = 0.2;
  contact_schedule.reset(t, {true, false, false, true});
  for (int i = 0; i < N + 1; ++i) {
    EXPECT_EQ(contact_schedule.phase(i), 0);
  }
  EXPECT_EQ(contact_schedule.numActiveContacts(0), 2);
  EXPECT_TRUE(contact_schedule.isContactActive(0)[0]);
  EXPECT_FALSE(contact_schedule.isContactActive(0)[1]);
  EXPECT_FALSE(contact_schedule.isContactActive(0)[2]);
  EXPECT_TRUE(contact_schedule.isContactActive(0)[3]);

  const double t1 = t + 0.15;
  contact_schedule.push_back(t1, {true, true, true, false});
  for (int i = 0; i < N + 1; ++i) {
    if (t + dt * i < t1) {
      EXPECT_EQ(contact_schedule.phase(i), 0);
    } else {
      EXPECT_EQ(contact_schedule.phase(i), 1);
    }
  }

  EXPECT_EQ(contact_schedule.numActiveContacts(0), 2);
  EXPECT_TRUE(contact_schedule.isContactActive(0)[0]);
  EXPECT_FALSE(contact_schedule.isContactActive(0)[1]);
  EXPECT_FALSE(contact_schedule.isContactActive(0)[2]);
  EXPECT_TRUE(contact_schedule.isContactActive(0)[3]);

  EXPECT_EQ(contact_schedule.numActiveContacts(1), 3);
  EXPECT_TRUE(contact_schedule.isContactActive(1)[0]);
  EXPECT_TRUE(contact_schedule.isContactActive(1)[1]);
  EXPECT_TRUE(contact_schedule.isContactActive(1)[2]);
  EXPECT_FALSE(contact_schedule.isContactActive(1)[3]);
}

} // namespace convexmpc

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
