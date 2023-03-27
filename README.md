# ROS-Neuro Recorder package
The package provides a node to read the incoming [NeuroFrame](https://github.com/rosneuro/rosneuro_msgs) and [NeuroEvent](https://github.com/rosneuro/rosneuro_msgs) messages and store them in a GDF file. 
The package relies on the library [libxdffileio](https://neuro.debian.net/pkgs/libxdffileio-dev.html) available in the NeuroDebian repository.

## Requirements
**rosneuro_recorder** has been tested with the following configuration:

- Ubuntu 18.04.5 LTS Bionic Beaver and ROS Melodic
- Ubuntu 20.04.6 LTS Focal Fossa and ROS Noetic

**rosneuro_recorder** depends on:

- [libxdffileio](https://neuro.debian.net/pkgs/libxdffileio-dev.html)
- [rosneuro/rosneuro_msgs](https://github.com/rosneuro/rosneuro_msgs)
- [rosneuro/rosneuro_data](https://github.com/rosneuro/rosneuro_data)

## Usage
The package reads new **NeuroFrame** and **NeuroEvent** messages ([rosneuro_msgs](https://github.com/rosneuro/rosneuro_msgs)) published in the related topics
and it stores them as GDF file. The node waits for the first **NeuroFrame** message to setup the data structure and to start the recording.

To launch the **rosneuro_recorder**, the following commands can be used:
```
rosrun rosneuro_recorder recorder _filename:=[FILENAME] _filepath:=[FILEPATH]
```

## Subscribed Topics
- ```/neurodata``` ([rosneuro_msgs/NeuroFrame](https://github.com/rosneuro/rosneuro_msgs))
- ```/events/bus``` ([rosneuro_msgs/NeuroEvent](https://github.com/rosneuro/rosneuro_msgs))

## Parameters
```~/autostart``` (```bool```, default: ```False```)

Automatically start the recorder after launch

```~/filename``` (```string```, default: ```UNKNOWN.YYYYMMDD.HHmmSS.neuromodality.neurotask.gdf```)

The GDF filename. Default filename is in the format: ```SUBJECT.DATE.TIME.MODALITY.TASK.EXTRA.gdf```.

```~/filepath``` (```string```, default: ```$HOME```)

The full path where to save the GDF file. By default the file is saved in the ```$HOME``` directory or in ```$ROSNEURO_DATA``` 
directory if such environment variable is provided.

```~/protocol/subject``` (```string```, default: ```UNKNONWN```)

The subject identifier.

```~/protocol/modality``` (```string```, default: ```neuromodality```)

The modality of the experiment (e.g., ```calibration```)

```~/protocol/task`` (```string```, default: ```neurotask```)

The task performed by the user (e.g., ```mi_bhbf``` for motor imagery, both hands vs. both feet)

```~/protocol/extra``` (```string```, default: ```empty```)

Extra field to describe the experiment (e.g., ```wheelchair```)

## Services

```~/record``` (```std_srvs/Empty```)

Start the recording

```~/quit``` (```std_srvs/Empty```)

Quit the recording

```~/get_info``` (```rosneuro_msgs/GetAcquisitionInfo``)

Get the configuration of the device

## Launcher
The package comes with a launcher (```recorder.launch```) that takes care of running the [rosneuro_acquisition](https://github.com/rosneuro/rosneuro_acquisition)
and the recorder. For instance:

```
roslaunch rosneuro_recorder recorder.launch plugin:=rosneuro::EGDDevice devarg:=eego filename:=TEST.20230327.135055.calibration.mi_bhbf.gdf
```
