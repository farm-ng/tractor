We have a few standard apriltag grid boards appropriate for intrinsic
and multi-camera rig calibration.  You can purchase physical rigid
boards here: https://farm-ng.com/store/p/calibration-board


This folder contains configuration files corresponding to the farm-ng
supplied calibration boards.  These configuration files will be
installed for use by the calibration applications by default.  You may
override the values, or generate your own board config files depending
on what you have on hand for calibration boards.


We chose the following IDs so as not to overlap with tag ids 429 and
below, which we like to reserve for other applications.  In addition,
we like having two independent sets of ids so that two calibration
boards may be used in the same scene without conflict.  For large
camera rigs, having two calibration boards in view allows us to
collect calibration datasets faster.

For tag ids 430-506, the config file was generated with the following line:
```
python -m farm_ng.calibration.apriltag_board_gen --root_tag_id=430 --name=board_430_506 --tag_size=0.044 --num_cols=7 --num_rows=11 --tag_spacing=0.056 > board_430_506.json
```


For tag ids 507-583, the config file was generated with the following line:
```
python -m farm_ng.calibration.apriltag_board_gen --root_tag_id=507 --name=board_507_583 --tag_size=0.044 --num_cols=7 --num_rows=11 --tag_spacing=0.056 > board_507_583.json
```
