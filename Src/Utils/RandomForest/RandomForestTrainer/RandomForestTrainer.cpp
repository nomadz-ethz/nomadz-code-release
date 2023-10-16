/*
 * Random Forest Trainer for Annotated Images
 *
 *  Created on: Nov 16, 2016
 *      Author: Florian Amstutz
 */

#include <iostream>
#include <fstream>
#include <iomanip>
#include <algorithm>
#include <dirent.h>
#include <sys/stat.h>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "Util/ezOptionParser.hpp"

#include "Patch.h"
#include "Tree.h"
#include "TreeBuilder.h"
#include "TreeReadWrite.h"

#include "SplitFunctions.h"
#include "StatisticFunctions.h"
#include "CostFunctions.h"

using namespace std;
using namespace cv;

#define VERSION 3

// Thanks https://stackoverflow.com/questions/874134/find-if-string-ends-with-another-string-in-c#874160
inline bool ends_with(string const& value, string const& ending) {
  if (ending.size() > value.size())
    return false;
  return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

bool readDir(const string& path, vector<Patch>& patches, int labelClass) {
  DIR* dirp = opendir(path.c_str());
  if (!dirp)
    return false;

  patches.clear();

  struct dirent* dp;
  while ((dp = readdir(dirp)) != NULL) {
    // Must be a regular file
    if (dp->d_type != DT_REG)
      continue;

    // Must end with .bmp
    const string filename(dp->d_name);
    if (!ends_with(filename, ".bmp"))
      continue;

    // Find the image
    Patch patch;
    patch.name = filename;
    patch.label = labelClass;
    patch.mat = cv::imread(path + "/" + filename);

    // Ignore if an image cannot be read
    if (!patch.mat.data) {
      cerr << "Warning: Could not open or find the image " << patch.name << endl;
      continue;
    }

    // Convert image from BGR to RGB or CbCrY to YcrCb
    cvtColor(patch.mat, patch.mat, COLOR_BGR2RGB);

    patches.push_back(patch);
  }

  return (patches.size() != 0);
}

string getDir(const string& filepath) {
  string pathInputData;
  string tempPath = filepath;
  string tempFolder;
  for (int i = 0; i < tempPath.size(); i++) {
    tempFolder = tempFolder + tempPath[i];

    if (tempFolder == "xml") {
      break;
    }

    if (tempPath[i] == '/') {
      pathInputData = pathInputData + tempFolder;
      tempFolder.clear();
    }
  }
  if (pathInputData.size() > 0)
    return pathInputData;
  else
    return filepath;
}

/*
 * Read and parse the xml files
 */
bool readXML(const string& filename, vector<Patch>& currentPatches, int labelClass) {
  // Clear currentPatches
  currentPatches.clear();

  // Open xml file
  FileStorage fs(filename, FileStorage::READ);
  if (!fs.isOpened())
    return false;

  // Iterate over all patches
  FileNode node = fs["Patches"];
  FileNodeIterator it = node.begin(), it_end = node.end();

  // Extract path input data
  string pathInputData = getDir(filename);

  for (; it != it_end; ++it) {
    Patch temp;

    temp.name = (string)(*it)["Name"];
    temp.label = labelClass;

    string tempName = pathInputData + temp.name;
    temp.mat = imread(tempName);

    // Convert image from BGR to RGB or CbCrY to YcrCb
    cvtColor(temp.mat, temp.mat, COLOR_BGR2RGB);

    // Check for invalid input
    if (!temp.mat.data) {
      cerr << "Could not open or find the image " << temp.name << endl;
      exit(0);
      return -1;
    }

    temp.diameter = (int)(*it)["Diameter"];
    (*it)["Offset"] >> temp.offset;

    FileNode cameraMatrix = (*it)["CameraMatrix"];
    cameraMatrix["Rotation"] >> temp.rotation;
    cameraMatrix["Translation"] >> temp.translation;

    currentPatches.push_back(temp);
  }

  fs.release();

  return true;
}

/*
 * Read patches from a given path (call readXML or readDir)
 */

bool readPatches(const string& path, vector<Patch>& patches, int labelClass) {
  struct stat s;
  if (stat(path.c_str(), &s) == 0) {
    // path is a regular file
    if (S_ISREG(s.st_mode))
      return readXML(path, patches, labelClass);

    // path is a directory
    else if (S_ISDIR(s.st_mode))
      return readDir(path, patches, labelClass);

    // path exists, but is not a directory or a file (see `man 2 stat` for full list!)
    else
      return false;

  } else {
    // path doesn't exist
    return false;
  }
}

void resizePatches(vector<Patch>& patches, int size, int interpolation = INTER_CUBIC) {
  for (int i = 0, n = patches.size(); i < n; ++i) {
    Mat& mat = patches[i].mat;
    cv::resize(mat, mat, cv::Size(size, size), 0, 0, interpolation);
  }
}

void savePatches(const vector<Patch>& patches, const string& dir) {
  mkdir(dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
  for (int i = 0, n = patches.size(); i < n; ++i) {
    const string& name = /*"cl" + to_string(patches[i].label) + "-" +*/ patches[i].name;
    const Mat& mat = patches[i].mat;
    Mat converted;
    cvtColor(mat, converted, COLOR_BGR2RGB);
    cv::imwrite(dir + "/" + name, converted);
  }
}

/*
 *Prepare data for training
 */
void prepareData(float& validationPercentage,
                 vector<vector<Patch>>& patchesTraining,
                 vector<vector<int>>& labelsTraining,
                 vector<Patch>& patchesValidation,
                 vector<int>& labelsValidation,
                 vector<string>& inputFilesTraining,
                 vector<string>& inputFilesValidation,
                 int& numberTrees,
                 int& maxNumberPatches,
                 bool validationSetAvailable,
                 string statistic_path) {
  // Clear all vectors
  patchesTraining.clear();
  labelsTraining.clear();
  patchesValidation.clear();
  labelsValidation.clear();

  // Check validation percentage
  if (validationPercentage >= 1 || validationPercentage <= 0) {
    cout << "Wrong validation percentage, taking 0,1" << endl;
    validationPercentage = 0.1;
  }

  // Temporary vector
  vector<Patch> tempPatches;

  // TRAINING SET------------------
  // Vector of vector of patches
  vector<vector<Patch>> patchesTrainingTemp;
  unsigned int maxPatchesTraining = 0;

  // Iterate over all xml files of training set
  for (int i = 0; i < inputFilesTraining.size(); i++) {
    // Read patches of current file into vector and add to patches vector
    if (!readPatches(inputFilesTraining.at(i), tempPatches, i)) {
      cerr << "Error while reading " << inputFilesTraining.at(i) << endl;
      exit(0);
    }

    patchesTrainingTemp.push_back(tempPatches);

    if (i == 0)
      maxPatchesTraining = tempPatches.size();

    else if (patchesTrainingTemp.at(i).size() < maxPatchesTraining)
      maxPatchesTraining = tempPatches.size();

    cout << "Number of patches read from file " << inputFilesTraining.at(i) << " = " << patchesTrainingTemp.at(i).size()
         << ", assigned label: " << i << endl;

    //@simonzi
    ofstream outfile;
    outfile.open(statistic_path, std::ios_base::app);
    outfile << "Number of patches read from file " << inputFilesTraining.at(i) << " = " << patchesTrainingTemp.at(i).size()
            << ", assigned label: " << i << endl;
    outfile.close();
  }

  // VALIDATION SET------------------
  if (validationSetAvailable) {
    // Vector of vector of patches
    vector<vector<Patch>> patchesValidationTemp;
    unsigned int maxPatchesValidation = 0;

    // Iterate over all xml files of validation set
    for (int i = 0; i < inputFilesValidation.size(); i++) {
      // Read patches of current file into vector and add to patches vector
      if (!readPatches(inputFilesValidation.at(i), tempPatches, i)) {
        cerr << "Error while reading " << inputFilesValidation.at(i) << endl;
        exit(0);
      }
      patchesValidationTemp.push_back(tempPatches);

      if (i == 0)
        maxPatchesValidation = tempPatches.size();

      else if (patchesValidationTemp.at(i).size() < maxPatchesValidation)
        maxPatchesValidation = tempPatches.size();

      cout << "Number of patches read from file " << inputFilesValidation.at(i) << " = "
           << patchesValidationTemp.at(i).size() << ", assigned label: " << i << endl;

      //@simonzi
      ofstream outfile;
      outfile.open(statistic_path, std::ios_base::app);
      outfile << "Number of patches read from file " << inputFilesValidation.at(i) << " = "
              << patchesValidationTemp.at(i).size() << ", assigned label: " << i << endl;
      outfile.close();
    }

    // Number of validation patches
    unsigned int numberValidationPatches = std::min<unsigned int>(
      maxPatchesValidation,
      std::min(ceil(validationPercentage * maxPatchesTraining), ceil(validationPercentage * maxNumberPatches)));

    // Create validation patches
    for (int i = 0; i < inputFilesValidation.size(); i++) {
      random_shuffle(patchesValidationTemp.at(i).begin(), patchesValidationTemp.at(i).end());
      patchesValidation.insert(patchesValidation.end(),
                               patchesValidationTemp.at(i).begin(),
                               patchesValidationTemp.at(i).begin() + numberValidationPatches);
    }

    // Create labelsValidation vector
    for (int i = 0; i < patchesValidation.size(); i++) {
      labelsValidation.push_back(patchesValidation.at(i).label);
    }
  }

  // Create training vector for each trees
  for (int i = 0; i < numberTrees; i++) {
    // Put all samples in one vector and shuffle
    vector<Patch> allPatchesTraining;

    int maxSamples = maxPatchesTraining;
    if (maxNumberPatches > 0)
      maxSamples = std::min(maxNumberPatches, (int)(maxPatchesTraining));

    // Create single vector of patches
    for (int j = 0; j < inputFilesTraining.size(); j++) {
      // Shuffle patches of class i before selecting subset
      random_shuffle(patchesTrainingTemp.at(j).begin(), patchesTrainingTemp.at(j).end());
      allPatchesTraining.insert(
        allPatchesTraining.end(), patchesTrainingTemp.at(j).begin(), patchesTrainingTemp.at(j).begin() + maxSamples);
    }

    // Shuffle vector with all patches
    random_shuffle(allPatchesTraining.begin(), allPatchesTraining.end());

    // Create label vector
    vector<int> labels;

    for (int j = 0; j < allPatchesTraining.size(); j++)
      labels.push_back(allPatchesTraining.at(j).label);

    // Add patch and label vector to set
    patchesTraining.push_back(allPatchesTraining);
    labelsTraining.push_back(labels);
  }
}

/*
 * Option Parser
 */
int parseOptions(int argc,
                 const char** argv,
                 vector<string>& inputFilesTraining,
                 vector<string>& inputFilesValidation,
                 vector<string>& outputFiles,
                 int& maxDepth,
                 int& minSamplesSplit,
                 int& minSamplesLeaf,
                 int& numberRandomTests,
                 bool& verbose,
                 int& maxNumberPatches,
                 int& patchSize) {

  // Create Optionparser
  ez::ezOptionParser opt;

  opt.overview = "\nRandom Forest Trainer v" + to_string(VERSION);
  opt.syntax = "RandomForestTrainer [optIONS]";
  opt.example = "RandomForestTrainer -d 5 -s 20 -l 10 -r 1000 -a 16 -i trainingSet/ball.xml:trainingSet/background.xml -j "
                "validationSet/ball.xml:validationSet/background.xml -o tree0.xml:tree1.xml -v\n\n";
  opt.footer = "2016, Florian Amstutz\n\n";

  // Help
  opt.add("",                            // Default.
          0,                             // Required?
          0,                             // Number of args expected.
          0,                             // Delimiter if expecting multiple args.
          "Display usage instructions.", // Help description.
          "-h",                          // Flag token.
          "-help",                       // Flag token.
          "--help",                      // Flag token.
          "--usage"                      // Flag token.
  );
  // Input files training
  opt.add("", 1, 1, ':', "Input Training: At least two from the annotation tool created xml files seperated by :", "-i");
  // Input files validation
  opt.add("", 1, 1, ':', "Input Validation: Corresponding validation set to training set seperated by :", "-j");
  // Output file
  opt.add("", 1, 1, ':', "Output: Paths for saving the trees as xml seperated by :  ", "-o");
  // Maximum tree depth
  opt.add("", 0, 1, 0, "Maximum depth of random tree (default: 10)", "-d");
  // Minimum samples leaf
  opt.add("", 0, 1, 0, "The minimum number of samples required to be at a leaf node (default: 10)", "-l");
  // Minimum samples for split
  opt.add("", 0, 1, 0, "The minimum number of samples required to split an internal node (default: 20)", "-s");
  // Number of random tests per node
  opt.add("", 0, 1, 0, "Number of random tests per node (default: 1000)", "-r");
  // Maximum number of patches
  opt.add("", 0, 1, 0, "Maximum number of patches/samples for each class (default: smallest number of patches)", "-p");
  // Patch size
  opt.add("", 0, 1, 0, "Size of patches (default: 16)", "-a");
  // Verbose mode
  opt.add("", 0, 0, 0, "Verbose mode. Provides additional output.", "-v");

  opt.parse(argc, argv);

  string usage;

  // Check whether args are missing
  vector<string> badoptions;
  if (!opt.gotRequired(badoptions)) {
    for (int i = 0; i < badoptions.size(); ++i)
      std::cerr << "ERROR: Missing required option " << badoptions[i] << ".\n";

    opt.getUsage(usage);
    cout << usage;
    return 1;
  }

  // Print usage if -h option set
  if (opt.isSet("-h")) {
    opt.getUsage(usage);
    cout << usage;
    return 1;
  }

  // Check if more then one -i option and get input paths
  if (opt.isSet("-i")) {

    opt.get("-i")->getStrings(inputFilesTraining);

    if (inputFilesTraining.size() < 2) {
      cerr << "ERROR: At least two input files are needed!"
           << "\n";
      opt.getUsage(usage);
      cout << usage;
      return 1;
    }
  }

  // Check output parameter
  if (opt.isSet("-j")) {
    opt.get("-j")->getStrings(inputFilesValidation);
  }
  if (opt.isSet("-o")) {
    opt.get("-o")->getStrings(outputFiles);
  }
  if (opt.isSet("-d")) {
    opt.get("-d")->getInt(maxDepth);
  }
  if (opt.isSet("-s")) {
    opt.get("-s")->getInt(minSamplesSplit);
  }
  if (opt.isSet("-l")) {
    opt.get("-l")->getInt(minSamplesLeaf);
  }
  if (opt.isSet("-r")) {
    opt.get("-r")->getInt(numberRandomTests);
  }
  if (opt.isSet("-v")) {
    verbose = true;
  }
  if (opt.isSet("-p")) {
    opt.get("-p")->getInt(maxNumberPatches);
  }
  if (opt.isSet("-a")) {
    opt.get("-a")->getInt(patchSize);
  }
  return 0;
}

/*
 *Main function
 */
int main(int argc, const char** argv) {

  // Input vector training
  vector<string> inputFilesTraining;
  // Input vector validation
  vector<string> inputFilesValidation;
  // Output vector
  vector<string> outputFiles;
  // Maximum depth of tree
  int maxDepth = 10;
  // Minimum samples for split
  int minSamplesSplit = 20;
  // Minimum samples at leaf
  int minSamplesLeaf = 10;
  // Number of random tests per node
  int numberRandomTests = 1000;
  // Verbose mode
  bool verbose = false;
  // Maximum number of patches for each class
  int maxNumberPatches = -1;
  // Patch size
  int patchSize = 16;

  // Parse options
  int error = parseOptions(argc,
                           argv,
                           inputFilesTraining,
                           inputFilesValidation,
                           outputFiles,
                           maxDepth,
                           minSamplesSplit,
                           minSamplesLeaf,
                           numberRandomTests,
                           verbose,
                           maxNumberPatches,
                           patchSize);

  // Return error
  if (error)
    return error;

  // Check if validation set available
  bool validationSetAvailable = true;
  FileStorage fs_temp(inputFilesValidation.at(0), FileStorage::READ);
  if (!fs_temp.isOpened()) {
    validationSetAvailable = false;
    cout << endl;
    cout << "No validation set available. Create trees without validation!" << endl;
  } else {
    fs_temp.release();
  }

  // Create path to statistics file
  string tree_folder;
  int treeFolderIter = 0;
  while (outputFiles.at(0)[treeFolderIter] != '/') {
    tree_folder.push_back(outputFiles.at(0)[treeFolderIter]);
    treeFolderIter++;
  }
  string statistic_file = "/RandomForestStatistics.txt";
  string statistic_path = tree_folder + statistic_file;

  cout << endl << "Random Forest Trainer v" << VERSION << endl << "------------------------" << endl;

  //@simonzi - Write parameters into txt file
  ofstream outfile;
  outfile.open(statistic_path, std::ios_base::app);
  outfile << endl << "Random Forest Trainer v" << VERSION << endl << "------------------------" << endl;
  outfile << "inputFilesTraining "
          << "-i ";
  for (int i = 0; i < inputFilesTraining.size(); i++) {
    outfile << inputFilesTraining[i] << " ";
  }
  outfile << endl;
  if (validationSetAvailable) {
    outfile << "inputFilesValidation "
            << "-j ";
    for (int i = 0; i < inputFilesValidation.size(); i++) {
      outfile << inputFilesValidation[i] << " ";
    }
    outfile << endl;
  } else {
    outfile << "No validation set used." << endl;
  }
  outfile << "outputFiles "
          << "-o ";
  for (int i = 0; i < outputFiles.size(); i++) {
    outfile << outputFiles[i] << " ";
  }
  outfile << endl;
  outfile << "maxDepth "
          << "-d " << maxDepth << endl;
  outfile << "minSamplesSplit "
          << "-s " << minSamplesSplit << endl;
  outfile << "minSamplesLeaf "
          << "-l " << minSamplesLeaf << endl;
  outfile << "numberRandomTests "
          << "-r " << numberRandomTests << endl;
  outfile << "maxNumberPatches "
          << "-p " << maxNumberPatches << endl;
  outfile << "patchSize "
          << "-a " << patchSize << endl;
  outfile << endl;
  outfile.close();

  // initialize random seed:
  srand(time(NULL));

  // Prepare the data
  vector<vector<Patch>> patchesTraining;
  vector<vector<int>> labelsTraining;
  vector<Patch> patchesValidation;
  vector<int> labelsValidation;
  float validationPercentage = 0.05;

  int numberClasses = inputFilesTraining.size();
  int numberTrees = outputFiles.size();

  // prepare training set
  prepareData(validationPercentage,
              patchesTraining,
              labelsTraining,
              patchesValidation,
              labelsValidation,
              inputFilesTraining,
              inputFilesValidation,
              numberTrees,
              maxNumberPatches,
              validationSetAvailable,
              statistic_path);

  int numberPatchesTraining = patchesTraining.size();
  int numberPatchesValidation = patchesValidation.size();

  // Create TreeBuilder
  PixelDifference prototypeSplit(patchSize);
  TreeBuilder<Patch, PixelDifference, LeafStatistic, EntropyFunction> builder(
    prototypeSplit, numberClasses, maxDepth, numberRandomTests, minSamplesLeaf, minSamplesLeaf);
  cout << endl << "Starting training..." << endl;

  // Vector for trees
  vector<DecisionTree<Patch, LeafStatistic>> trees;

  // Build the trees
  for (int i = 0; i < numberTrees; i++) {
    cout << "Building tree " << (i + 1) << " of " << numberTrees << endl;
    trees.push_back(builder.train_tree(patchesTraining.at(i), labelsTraining.at(i)));
  }

  cout << "Finished building trees!" << endl;

  if (validationSetAvailable) {
    cout << "Starting validation..." << endl;

    // Define confusion matrix
    vector<vector<int>> predictionMatrix(numberClasses, vector<int>(numberClasses));

    // Validate tree with validation patches
    for (int i = 0; i < numberPatchesValidation; i++) {
      int predicted = 0;
      float predictedProbability = 0;
      float currentProbability;

      // Iterate over all probabilities
      for (int j = 0; j < numberClasses; j++) {
        currentProbability = 0;

        // Iterate over all trees
        for (int k = 0; k < numberTrees; k++) {
          // Get Leaf statistic
          currentProbability += trees.at(k).eval(patchesValidation.at(i)).probabilities()[j];
        }
        currentProbability = currentProbability / numberTrees;

        // Check whether current probability is bigger than biggest seen so far
        if (currentProbability > predictedProbability) {
          predicted = j;
          predictedProbability = currentProbability;
        }
      }

      // Print FP prediction if verbose mode
      if (verbose && predicted != labelsValidation.at(i)) {
        // cout << "False prediction for patch " << patchesValidation.at(i).name << ": label " << predicted << " instead of "
        // << labelsValidation.at(i) << endl;
      }
      // Fill confusion matrix
      predictionMatrix[predicted][labelsValidation.at(i)]++;
    }

    // Calculate Precision, Recall, Accuracy

    float* precision = new float[numberClasses]();
    float* recall = new float[numberClasses]();
    float accuracy = 0;
    int TP, FP, FN, TN;

    for (int i = 0; i < numberClasses; i++) {

      TP = predictionMatrix[i][i];
      FP = FN = TN = 0;
      accuracy += TP;

      for (int j = 0; j < numberClasses; j++) {
        if (i != j) {
          FN += predictionMatrix[j][i];
          FP += predictionMatrix[i][j];
        }
      }
      precision[i] = (TP + 0.0f) / (TP + FP);
      recall[i] = (TP + 0.0f) / (TP + FN);
    }

    accuracy = accuracy / numberPatchesValidation;

    cout << "Finished!" << endl << endl;

    // Draw Confusion Matrix

    cout << endl << "Confusion Matrix:" << endl << endl;

    ostringstream tableTop;
    ostringstream tableMiddle;
    ostringstream tableBottom;

    tableTop << setw(16) << setfill('-') << "+" << setw(((9 * numberClasses - 13) / 2)) << setfill(' ') << " "
             << "Ground truth" << setw((ceil((9 * numberClasses - 13) / 2.0f))) << setfill(' ') << " "
             << "+";
    tableTop << setw(10) << setfill('-') << "+" << endl;
    tableTop << setw(16) << setfill(' ') << std::right << "Predicted Class|";

    tableBottom << setw((16 + numberClasses * 9 + 10)) << setfill('=') << "=" << endl;
    tableBottom << setw(15) << setfill(' ') << std::right << "Recall"
                << "|";

    // Extract name of classes
    vector<string> classNames(numberClasses, " ");
    string tempFolder;
    for (int i = 0; i < numberClasses; i++) {
      string tempPath = inputFilesTraining.at(i);

      for (int j = 0; j < tempPath.size(); j++) {
        if (tempPath[j] == '.') {
          classNames[i] = tempFolder;
          break;
        }

        tempFolder = tempFolder + tempPath[j];

        if (tempPath[j] == '/') {
          tempFolder.clear();
        }
      }
    }

    for (int i = 0; i < numberClasses; i++) {

      // print ground truth label
      // tableTop << setw(8) << setfill(' ') << std::right << i << "|";
      tableTop << setw(8) << setfill(' ') << std::right << classNames[i] << "|";
      // Draw new line for predicted class
      tableMiddle << setw(16) << setfill('-') << "+";
      // Draw line under ground truth
      for (int i = 0; i < numberClasses; i++)
        tableMiddle << setw(9) << setfill('-') << "+";
      // Draw line under precision
      tableMiddle << setw(10) << setfill('-') << "+";
      // Print predicted class
      tableMiddle << endl
                  //<< setw(15) << setfill(' ') << std::right << i << "|";
                  << setw(15) << setfill(' ') << std::right << classNames[i] << "|";

      // Fill matrix
      for (int j = 0; j < numberClasses; j++)
        tableMiddle << setw(8) << setfill(' ') << std::right << predictionMatrix[i][j] << "|";
      // Print precision and recall
      tableMiddle << setw(9) << setfill(' ') << std::right << fixed << setprecision(3) << precision[i] << "|" << endl;
      tableBottom << setw(8) << setfill(' ') << std::right << fixed << setprecision(3) << recall[i] << "|";
    }
    tableTop << setw(10) << setfill(' ') << std::right << "Precision|";

    // Print table
    cout << tableTop.str() << endl << tableMiddle.str() << tableBottom.str() << endl << endl;

    // print accuracy
    cout << "Accuracy: " << fixed << setprecision(3) << accuracy << endl << endl;

    //@simonzi - Write results of confusion matrix into txt file
    outfile.open(statistic_path, std::ios_base::app);

    outfile << endl << "Confusion Matrix:" << endl << endl;

    outfile << tableTop.str() << endl << tableMiddle.str() << tableBottom.str() << endl << endl;

    outfile << "Accuracy: " << fixed << setprecision(3) << accuracy << endl << endl;

    outfile << "SUMMARY: " << endl;
    outfile << "Label"
            << " "
            << "Precision"
            << " "
            << "Recall"
            << " "
            << "Accuracy" << endl;

    for (int i = 0; i < numberClasses; i++) {
      outfile << i << " " << precision[i] << " " << recall[i] << " " << accuracy << endl;
    }

    outfile << endl;
    outfile.close();
  }

  // Save the trees to xml
  for (int i = 0; i < numberTrees; i++) {
    TreeReadWrite<Patch, LeafStatistic, PixelDifference>::writeTree(trees.at(i), outputFiles.at(i));
    cout << "A Tree has been saved to " << outputFiles.at(i) << endl;
  }
  cout << endl;

  return 0;
}
