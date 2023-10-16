#!/bin/bash
#Specifiy input files
PATH_INPUT_FILES="/media/simonzi/7a918d10-5a17-4a68-ad92-72add00476f3/home/simonzi/Documents/RoboCup/RandomForest/AnnotationTool"

NUM_CLASSES=3 #WITHOUT negative!

CLASS_1="ball"
CLASS_2="robotBody"
CLASS_3="line"
CLASS_4="dummy"
CLASS_5="dummy"
CLASS_6="dummy"
CLASS_7="dummy"
CLASS_8="dummy"
CLASS_9="dummy"

#Number of trees and forests which have to be created
NUMBER_TREES=9
NUMBER_FORESTS=6

FOREST_1="BallUpper1"
FOREST_2="BallUpper2"
FOREST_3="BallUpper3"
FOREST_4="BallUpper4"
FOREST_5="BallLower1"
FOREST_6="BallLower2"

#Maximum tree depth, minimum samples for splitting, minimum samples at leaf node and number of random tests per node
MAX_DEPTH=10
MIN_SAMPLES_SPLIT=20
MIN_SAMPLES_LEAF=10
NUMBER_RANDOM_TESTS=1000
MAXIMUM_NUMBER_PATCHES=10000 #For training set (validation set automatically 5% of this)

PATCH_SIZE_1=38
PATCH_SIZE_2=28
PATCH_SIZE_3=16
PATCH_SIZE_4=10
PATCH_SIZE_5=32
PATCH_SIZE_6=26

VERBOSE=''

#------------------------------------------------------------------------------

FOREST=1
while (( $FOREST <= $NUMBER_FORESTS ))
do
	case $FOREST in
		"1") FOLDER_TEMP=$FOREST_1;;
		"2") FOLDER_TEMP=$FOREST_2;;
		"3") FOLDER_TEMP=$FOREST_3;;
		"4") FOLDER_TEMP=$FOREST_4;;
		"5") FOLDER_TEMP=$FOREST_5;;
		"6") FOLDER_TEMP=$FOREST_6;;
	esac

	case $FOREST in
		"1") PATCH_SIZE_TEMP=$PATCH_SIZE_1;;
		"2") PATCH_SIZE_TEMP=$PATCH_SIZE_2;;
		"3") PATCH_SIZE_TEMP=$PATCH_SIZE_3;;
		"4") PATCH_SIZE_TEMP=$PATCH_SIZE_4;;
		"5") PATCH_SIZE_TEMP=$PATCH_SIZE_5;;
		"6") PATCH_SIZE_TEMP=$PATCH_SIZE_6;;
	esac

	INPUT_FILES_TRAINING="${PATH_INPUT_FILES}/trainingSet${FOLDER_TEMP}/xml/negative.xml"
	INPUT_FILES_VALIDATION="${PATH_INPUT_FILES}/validationSet${FOLDER_TEMP}/xml/negative.xml"
	COUNTER=1
	while (( $COUNTER <= $NUM_CLASSES ))
	do
	 case $COUNTER in
		"1") CLASS=$CLASS_1;;
		"2") CLASS=$CLASS_2;;
		"3") CLASS=$CLASS_3;;
		"4") CLASS=$CLASS_4;;
		"5") CLASS=$CLASS_5;;
		"6") CLASS=$CLASS_6;;
		"7") CLASS=$CLASS_7;;
		"8") CLASS=$CLASS_8;;
		"9") CLASS=$CLASS_9;;
	 esac

	 INPUT_FILES_TRAINING="${INPUT_FILES_TRAINING}:${PATH_INPUT_FILES}/trainingSet${FOLDER_TEMP}/xml/${CLASS}.xml"
	 INPUT_FILES_VALIDATION="${INPUT_FILES_VALIDATION}:${PATH_INPUT_FILES}/validationSet${FOLDER_TEMP}/xml/${CLASS}.xml"

	 COUNTER=$((COUNTER+1))
	done

	INPUT_FILES=$INPUT_FILES_TRAINING


	#Creates output file string
	OUTPUT_FOLDER="Trees${FOLDER_TEMP}"
	if [ -d "$OUTPUT_FOLDER" ]; then
		echo "Save your old trees and remove the folder 'Trees' afterwards!"
		exit
	else
		mkdir $OUTPUT_FOLDER
	fi

	OUTPUT_FILES="${OUTPUT_FOLDER}/tree0.xml"
	TREE=1
	while (( $TREE < $NUMBER_TREES ))
	do
	 OUTPUT_FILES="${OUTPUT_FILES}:${OUTPUT_FOLDER}/tree${TREE}.xml"
	 TREE=$((TREE+1))
	done
	#
	ARGUMENTS="-d $MAX_DEPTH -s $MIN_SAMPLES_SPLIT -l $MIN_SAMPLES_LEAF -r $NUMBER_RANDOM_TESTS -p $MAXIMUM_NUMBER_PATCHES -a $PATCH_SIZE_TEMP -i $INPUT_FILES_TRAINING -j $INPUT_FILES_VALIDATION -o $OUTPUT_FILES -v $VERBOSE"
	echo $ARGUMENTS
	./RandomForestTrainer $ARGUMENTS

	FOREST=$((FOREST+1))
done


