^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package tiago_moveit_config
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.0.15 (2016-07-08)
-------------------
* Merge branch 'add-titanium-collisions-with-ft' into 'cobalt-devel'
  add missing potential collisions with ft sensor frames
  See merge request !5
* add collisions with ft sensor
* Merge branch 'tiago_configs' into 'cobalt-devel'
  Added the 4 possible configurations of tiago_moveit_config
  See merge request !4
* Added the 4 possible configurations of tiago_moveit_config
* Contributors: Jordi Pages, Sam Pfeiffer, Victor Lopez

0.0.14 (2016-06-13)
-------------------
* Added necessary dependence to run moveit with a simulated or real robot
* Add disable collisions for force torque sensor
* Contributors: Sam Pfeiffer

0.0.13 (2016-06-01)
-------------------
* Added controllers for hand and gripper
* Contributors: Sam Pfeiffer

0.0.12 (2016-04-04)
-------------------
* Increase max speed of torso
* Contributors: Sam Pfeiffer

0.0.11 (2016-04-04)
-------------------
* Missing hand_palm_link in collision disables
* Contributors: Sam Pfeiffer

0.0.10 (2016-04-04)
-------------------
* Add disables in between hand finger links
  Without this, the robot will refuse to plan with closed hand
* Contributors: Sam Pfeiffer

0.0.9 (2016-03-31)
------------------
* Add disable collisions
  Using the generator.
  From:
  1300 / 2145 pairs disabled in tiago_titanium (845 enabled)
  To:
  2268 / 3096 pairs disabled in tiago_titanium (828 enabled)
* Add disable collisions
  Generated using https://gist.github.com/awesomebytes/18fe75b808c4c644bd3d a script that runs the urdf tree for adjacent links and checks for links without collision mesh to also disable the collision computation between them.
  From:
  (Generating matrix with max sampling density)
  329 / 465 pairs disabled in tiago_steel (136 enabled)
  To:
  754 / 873 pairs disabled in tiago_steel (119 enabled)
* Contributors: Sam Pfeiffer

0.0.8 (2016-03-18)
------------------
* Added impossible collision disabling between torso_fixed_column_link and arm_2_link
* Contributors: Sam Pfeiffer

0.0.7 (2016-03-18)
------------------
* Passing change to titanium too about torso_fixed_column_link collision with arm1 disabling
* Added another currently happening collision exception between torso_fixed_column_link and arm_1_link
* Contributors: Sam Pfeiffer

0.0.6 (2016-03-18)
------------------
* Add hand passive joints as passive
* added clear octomap and removed exceptions on collisions of arm wit hhead
* Contributors: Sam Pfeiffer

0.0.5 (2016-03-10)
------------------
* Refs #11489. Discard collisions between torsolinks
* Fix collisions with column
* Remove elements of prototype mobilebase
* Disable collision hand safety box <-> wrist mesh
* Add arm group + disable more internal hand collisions
* Contributors: Bence Magyar, jordi.pages@pal-robotics.com

0.0.4 (2015-05-20)
------------------
* Add hand_safety_box to the game!
* Disable more collisions between hand links
* Contributors: Bence Magyar

0.0.3 (2015-04-14)
------------------
* Fix gripper parts
* Add torso controller
* Separate configuration files for titanium and steel, launch files parametrized
* Contributors: Bence Magyar

0.0.2 (2015-01-20)
------------------
* Remove tiago_description dependency
* Contributors: Bence Magyar

0.0.1 (2015-01-20)
------------------
* Added configuration with arm controllers
* Initial version of tiago_moveit_config (no hand)
* Contributors: Sammy Pfeiffer
