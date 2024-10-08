# avvv_etsi

Autonomous Vehicle V2X Visualiser ETSI. Refer to the official documents for more information.

## Build

Take the steps as they're provided below in all sections:

Clone this repository:

```
git clone git@github.com:hoosh-ir/avvv_dm.git
```

Navigate to the root directory:
```
cd avvv_dm
```

### Analyser

Navigate back to `lib`:
```
cd ../..
```

Create Python3 venv:
```
python3 -m venv ./avvv_dm_venv
```

Activate the venv:
```
source avvv_etsi_venv/bin/activate
```

Navigate to the `analyser` directory:

```
cd ../../../analyser
```

Install the dependencies using pip:

```
pip3 install -r requirements.txt
```

### Visualiser

Build the geographic library. From the root directory of the repository, navigate to `lib/geographiclib-2.3`:

```
cd lib/geographiclib-2.3
```

Create a new directory for building:
```
mkdir build && cd build
```

Configure, build and install GeographicLib:

```
cmake ..
make
sudo make install
```

Navigate to the `visualiser` directory:

```
cd ../../visualiser
```

Resolve ROS2 dependencies using `rosdep`:

```
source /opt/ros/humble/setup.bash
rosdep update
rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO
```

Build the whole workspace using `colcon`. While still in the visualiser directory run:

```
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

### Launcher

Install Qt6 libraries and dependencies:

```
sudo apt install qt6-tools-dev qt6-wayland
```

Navigate to the `ui` directory located in the root directory:

```
cd ui/visually_dm_launcher
```

Build the application:

```
cmake -S . -B build
make -C build -j4
```

### Build Troubleshoot

- In case you run into a SetupToolsDeprecationWarning issue when building any of the ROS packages, you need to downgrade you `setuptools` Python package using:
```
pip3 install setuptools==58.2.0
```
To make sure you have the right version of setuptools installed, execute the following:
```
pip3 show setuptools
```

## Input Data

Basically, the analyser module takes in the ROSBAG and CSV files and extracts the network information in form of reports and graphs and generates a single ROSBAG file for exhibition purposes.

Considering all of your input files are ready and gathered in a single folder called, for instance, `input_files`, the structure of the input data should be such as the following:

```
input_files
├── csvs
│   ├── Free_OBU_1.csv
│   ├── Free_RSU_1.csv
│   ├── Free_RSU_2.csv
│   ├── Free_RSU_3.csv
│   ├── Free_RSU_4.csv
│   ├── Object_OBU_1.csv
│   ├── Object_RSU_1.csv
│   ├── Object_RSU_2.csv
│   ├── Object_RSU_3.csv
│   ├── Object_RSU_4.csv
│   ├── Signal_OBU_1.csv
│   ├── Signal_RSU_1.csv
│   └── Signal_RSU_2.csv
└── rosbags
    └── OBU_1
        ├── metadata.yaml
        └── OBU_1_0.db3
    ...
```
The `input_files` directory should be divided into 2 subdirectories called `csvs` and `rosbags` for comfort.

**Note**: The syntax of the file names must abide by the following rules. Unless, the application won't work properly:
    - For object info data: `Object_OBU_#.csv` or `Object_RSU_#.csv`
    - For freespace info data: `Free_OBU_#.csv` or `Free_RSU_#.csv`
    - For signal info data: `Signal_OBU_#.csv` or `Signal_RSU_#.csv`

Currently, the application only uses the ROSBAG files for OBU TF data. If you've got no OBU ROSBAG files ready, there is a built-in tool in the UI app, section "Preprocessor", you can use to create one.

The CSV files must contain data in the Cool4 standard format.

A sample input data is provided with the package that you can explore.

## Run

After everything is installed properly, run the programme using the launcher script provided. In the terminal run:

```
. launcher.sh
```

This will launch a UI app through which you'll be able to run everything.

### Usage

#### Preprocessor

After launching the UI, the first tab opens the preprocessor. The preprocessor tool is optionally used to create the initial ROSBAG files. So, you need to use it if and only if you haven't prepared the ROSBAGs beforehand. To use the preprocessor, you need to fill in the required lines as follows:
- OBU TF CSV: The absolute path to the OBU TF CSV file; as in `/path/to/csv_folder/TF_OBU_1.csv`
- OBU Freespace CSV: The absolute path to the OBU Freespace CSV file; as in `/path/to/csv_folder/Free_OBU_1.csv`
- OBU Signal CSV: The absolute path to the OBU Signal CSV file; as in `/path/to/csv_folder/Signal_OBU_1.csv`
- OBU Object CSV: The absolute path to the OBU Object CSV file; as in `/path/to/csv_folder/Object_OBU_1.csv`
- Output folder: The absolute path to save the initial ROSBAG file; as in `/path/to/rosbags/`
- OBU ID: The OBU ID according to the files you just chose; as in `OBU_1`

This will run the preprocessor package and create the initial ROSBAG file in the output directory specified. Make sure you're data is clean and abides by the standards to avoid any issues.

#### Analyser

To run the analyser on your input data, in the "Analyser" first fill the ROSBAGs and CSVs directories in the Analyser tab. To do so click on the "Browse" buttons and locate the `rosbags` and `csvs` directories and select "Choose". Then determine an output folder for your output files and click on the "Analyse" button. This will generate a single ROSBAG file along with all the graphs and report files.

#### Visualiser

If you ran the analyser immediately before this stage, the Visualiser's first tab, namely "ROSBAG", will automatically fill in the ROSBAG file it needs. Otherwise, if it's not filled, click on "Browse" and locate the generated ROSBAG by the Analyser, then click on "Analyse" below "Browse". Don't forget to click on "Analyse" at the bottom of the tab. This is necessary for correct handling of the topics inside the ROSBAG file.

In the Network tab, you can configure how the network will show up in the visualiser app. You can, for example, configure the RSU-OBU delay be visualised by the RSU-OBU link-colour in the visualiser and then select each colour to represent what value. You can configure the connection range of the RSU-OBU pairs. You can select to display real time graphs of the network characteristics by checking the box. Also you are able to check the offline heatmap box and select the offline heatmap, generated by Analyser, to display in Visualiser. At last you can check the online heatmap box to display the real time heatmaps of all network attributes for all RSU-OBU pairs. Don't forget to click "Apply" after changing the settings.

In "Maps", you are able to select Lanelet2 and/or Point Cloud maps with an arbitrary offset to show up in Visualiser. You might need to move RViz to your map's coordinates when visualisation is running.

In the last tab, "Launch", you can launch the visualiser app and play the selected ROSBAG and view the results in Visualiser. Some options are provided for playing the ROSBAG.

#### Reporter

The reporter tab is responsible to display generated pickle graphs by Analyser. Click on "Browse" and locate the generate `outputs` folder. Then locate `graphs` and then your desired RSU, OBU or RSU-OBU pair directory. Then locate `pickles` and select "Choose". Out of the list of graphs shown on the tab, select as many as you want. Clicking on "Display" should bring up their graphs.
