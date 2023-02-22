RPC_TEST=test
PATH_DRACO=/home/mingyo/Projects/draco3/rpc_mingyo
rm /tmp/draco_*
rm $PATH_DRACO/*mp4
python3 $PATH_DRACO/simulator/pybullet/draco_test.py --mode=headless
mkdir $PATH_DRACO/data/$RPC_TEST
mv $PATH_DRACO/*mp4 $PATH_DRACO/data/$RPC_TEST
mv /tmp/draco_* $PATH_DRACO/data/$RPC_TEST
python3 $PATH_DRACO/hand_pose.py --path=${PATH_DRACO}/data/${RPC_TEST}
