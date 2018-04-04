testlist=(
	'1000_syntax'
	'0000_syntax'
#	'0002_syntax'
	'0024_syntax'
	'0028_syntax'
	'002C_syntax'
	'0030_syntax'
	'0032_syntax'
	'0050_syntax'
	'1000_interrupt'
	'0000_interrupt'
#	'0002_interrupt'
	'0024_interrupt'
	'0028_interrupt'
	'0032_interrupt'
	'0050_interrupt'
	'1000_functional'
	'0000_functional'
	'0002_functional'
#	'0024_functional'
#	'0028_functional'
   )

test_index=-1
loop_count=1
test_list=0

for i in "$@"
do
case $i in
    --test-index=*)
    test_index="${i#*=}"
    shift # past argument=value
    ;;
    --test-list)
    test_list=1
    shift # past argument=value
    ;;
    --loop-count=*)
    loop_count="${i#*=}"
    shift # past argument=value
    ;;
    *)
    echo "Unrecognized parameter: '$i'."
    echo "Supported parameters: --test-index=<number>, --test-list, --loop-count=<number>"
    echo "Ex:"
    echo "Print out test list"
    echo "$ ./run_test.sh --test-list"
    echo "Run all test cases"
    echo "$ ./run_test.sh"
    echo "Run the #5 test case"
    echo "$ ./run_test.sh --test-index=5"
    echo "Run the cases 10 time"
    echo "$ ./run_test.sh --loop-count=10"
    echo "Run the cases forever"
    echo "$ ./run_test.sh --loop-count=0"
    exit 1
          # unknown option
    ;;
esac
done

function diff_expected {
	rm -f ./output &>/dev/null
	adb pull /data/output &>/dev/null
	if ! diff -q $1 ./output > /dev/null  2>&1; then
		echo "FAILED"
		echo "Expected:"
		cat $1
		echo
		echo "Actual:"
		cat ./output
		echo
	else
		echo "PASSED"
	fi
}

i=1
if [ "$test_list" == "1" ]
then
	for test in ${testlist[@]}; do
		echo "#$i	$test"
		let i=i+1
	done
	exit
fi

loop_forever=1
if [ $loop_count -lt 1 ]
then
loop_count=0
else
loop_forever=0
fi

echo "Start test"
adb push . /data/script &>/dev/null
adb shell "chmod 777 -R /data/script" &>/dev/null

i=1
while [  $loop_forever -eq 1 -o $i -le $loop_count ]; do
	echo "#### Start Test cycle $i ####"
	
	index=1;
	for test in ${testlist[@]}; do
		if [ $test_index -eq -1 -o $test_index -eq $index ]
		then
			echo "Running $test.sh"
			adb shell "rm -f /data/output"
			adb shell "/data/script/$test.sh" &>/dev/null
			diff_expected $test.exp
		fi
		let index=index+1
	done

	echo "#### End Test cycle $i ####"
	echo ""
	let i=i+1
done


