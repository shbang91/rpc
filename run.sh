RPC_TEST=test2
PATH_DRACO=.
python3 $PATH_DRACO/simulator/pybullet/draco_test.py
mkdir -p $PATH_DRACO/data/$RPC_TEST
mv $PATH_DRACO/*mp4 $PATH_DRACO/data/$RPC_TEST
mv /tmp/draco_* $PATH_DRACO/data/$RPC_TEST
python3 $PATH_DRACO/hand_pose.py --path=${PATH_DRACO}/data/${RPC_TEST}
