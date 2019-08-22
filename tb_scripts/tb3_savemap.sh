rosrun map_server map_saver -f ~/map
today=`date +%Y-%m-%d.%H:%M:%S`
cp ~/map.pgm ~/tb_maps/"map_$today.pgm"
cp ~/map.yaml ~/tb_maps/"map_$today.yaml"
echo "Map saved!"
