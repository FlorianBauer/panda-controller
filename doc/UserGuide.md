# User Guide

This guide provides a short descriptions of the available functionalities and how to use them. 
Instructions on how to build and run the software can be found in the [README](../README.md).


# Feature Overview


## `Plate Type Manager`

| Properties | Commands          |
|------------|-------------------|
| Plate Type | Delete Plate Type |
|            | Get Plate Type    |
|            | Set Plate Type    |

The `Plate Type Manager` is used to simplify the handling of various objects for the robot gripper.
Since the usage is focused on handling microplates, the term _Plate_ is used for every quadratic
object the gripper is able to hold.

With the `Set Plate Type`-command a alphanumerical ID, as well as the dimensions of an object can
be defined. The defined object is then stored in `.sila/panda_controller/plate_types` directory and
remain persistent even after an SiLA server reboot. The data is therefore stored in an simple JSON
format like shown below, where the dimension values are given in meters:
```
{
  "type": "conring_96_wellplate",
  "dimX": 0.12776,
  "dimY": 0.08547,
  "dimZ": 0.01422
}
```

The command `Get Plate Type` retrieves the dimensions of a given ID.

`Delete Plate Type` removes plate type with the given ID.

The `Plate Type`-property lists all defined plate types by their ID.


## `Site Manager`

| Properties    | Commands               |
|---------------|------------------------|
| Finger Length | Get Site               |
| Sites         | Set Site               |
|               | Remove Plate From Site |
|               | Set Finger Length      |
|               | Delete Site            |
|               | Is Site Occupied       |
|               | Put Plate On Site      |

The `Site Manager` is to abstract the handling of various locations inside the operational range of 
the robot arm. Therefore, a specific _Site_ can be defined with the `Set Site` command. A _Site_ 
consists of an alphanumerical ID used as handle, a location, a orientation and a move direction to 
approach the target point as well as a optional move direction to retrieve from the target point.

The Property `Sites` lists all defined site-IDs.

`Get Site` returns the given site and its values.

`Delete Site` removes the given site permanently.


All defined sites are stored in JSON files at the `.sila/panda_controller/sites` directory.

_Plates_ defined by the `Plate Type Manager` can be placed on a specific site. Therefore the 
command `Put Plate On Site` requires a plate-ID and the site-ID. The same applies for 
`Remove Plate From Site`. With `Is Site Occupied` the vacancy status can be checked.

With `Set Finger Lenght`, the length of the gripper fingers can be defined. This is necessary on 
modified or custom gripper extensions to allow the accurate pick up of objects which are placed on 
a site.


## `Robot Controller`

| Commands           |
|--------------------|
| Set Gripper Effort |
| Move To Pose       |
| Pick Plate         |
| Get Current Pose   |
| Follow Path        |
| Set To Frame       |
| Get Arm Effort     |
| Move To Site       |
| Place Plate        |
| Follow Frames      |
| Set Arm Effort     |
| Close Gripper      |
| Get Current Frame  |
| Transport Plate    |
| Move Relative      |
| Get Gripper Effort |
| Set Gripper        |

Commands in the `Robot Controller` feature are generally in charge of the actual movements. These 
can be distinguished in high and low level operations. High level operations such as `Pick Plate`, 
`Move To Site`, `Place Plate`, `Transport Plate` only require site-IDs and/or plate-IDs. In contrast 
to this, the low level operations allow direct and fine-grained usage of the robot and the gripper. 
The according parameters must therefore be set to full extent with all their tedious details.


# Collision Avoidance

The path planning of a movement is based on inverse kinematics (IK). The integrated IK-Solver 
relies on the accurate definition of collision objects within a 3D scene to avoid crashes with 
other objects in the range of the robot arm. These objects are not manageable within the SiLA 
features. However, a collision object (like a wall or a barrier) can be defined externally within 
JSON files. Therefore, a JSON file must be placed in the `.sila/panda_controller/collision_objects` 
directory. After defining such a file, the server must be restarted to recognize the change.

The file itself consist of the following fields:

| Field  | Description                                   |
|--------|-----------------------------------------------|
| `id`   | The ID of the collision object.               |
| `posX` | The `x` position of the object in m.          |
| `posY` | The `y` position of the object in m.          |
| `posZ` | The `z` (height) position of the object in m. |
| `rotP` | The pitch rotation in rad.                    |
| `rotR` | The roll rotation in rad.                     |
| `rotY` | The yaw rotation in rad.                      |
| `dimX` | The `x` dimension of the object in m.         |
| `dimY` | The `y` dimension of the object in m.         |
| `dimZ` | The `z` (height) dimension of the object in m.|


Example:
```
{
	"id": "wall",
	"posX": 0.79,
	"posY": -0.715,
	"posZ": 0.55,
	"rotP": 0.0,
	"rotR": 0.0,
	"rotY": 0.0,
	"dimX": 1.0,
	"dimY": 0.39,
	"dimZ": 2.5
}
```

