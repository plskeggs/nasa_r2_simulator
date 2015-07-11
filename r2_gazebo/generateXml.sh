for file in `rospack find r2_gazebo`/robots/*.xacro
do
    filename=${file%.*o}
    echo $file
    rosrun xacro xacro.py $file > "$filename.xml"
    mv "$filename.xml" `rospack find r2_gazebo`/robots
done
exit 0
