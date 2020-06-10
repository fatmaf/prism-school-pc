#starting with wh 
echo "Running warehouse 1 r"
./bin/testrunoptions.sh warehouse1 r
sleep 120
echo "Running warehouse 2 r"
./bin/testrunoptions.sh warehouse2 r 
sleep 120
echo "Running warehouse free r"
./bin/testrunoptions.sh warehousefree r 
sleep 120
echo "Running strangehouse r"
./bin/testrunoptions.sh strangehouse r 
sleep 120 
echo "Running grid 11 r"
./bin/testrunoptions.sh grid 11r 
sleep 120 
echo "Running grid fixed r"
./bin/testrunoptions.sh gfixed 2r
sleep 120

echo "Running grid 11 g"
./bin/testrunoptions.sh grid 11g
sleep 120
echo "Running warehouse free g"
./bin/testrunoptions.sh warehousefree g 
sleep 120
echo "Running grid fixed g"
./bin/testrunoptions.sh gfixed 2g
sleep 120 
echo "Running strangehouse g"
./bin/testrunoptions.sh strangehouse g
sleep 120 

echo "Running warehouse 2 g"
./bin/testrunoptions.sh warehouse2 g 
sleep 120
echo "Running warehouse 1 g"
./bin/testrunoptions.sh warehouse1 r

#echo "Running warehouse 1 g"
#./bin/testrunoptions.sh warehouse1 g
#sleep 120
echo "All Done"

