
# Change Log
All notable changes to this project will be documented in this file.
 
 
## [2024-02-07] 
 
Worked on april tags, mostly incomplete, pixel placing is still based on roadrunner pose estimate which is still very accurate.

### Added
- added rotational snap to the backboard
- added yellow pixel on backboard for Front-Red-Left
 
### Changed
- arm movement is now on left stick y
- improved PID values for Roadrunner
- improved Track Width values for Roadrunner


## [2024-02-02]
 
Added backstage side vision autonomous this uses very similar moves to the front with different coordinates, needed to modify the return trajectories a lot to avoid descoring any pixels on spike marks and to park in the tile closest to the center of the field in the backstage, this helps with teamwork by not making the other team on our alliance sad because our robot slammed into them at full speed trying to park or our robot completely descoring the pixel they placed on the spike mark.

### Added
- added vision autonomous for backstage side of field, very similar in functionality to the frontside autonomous but with new coordinates
 
### Changed
- improved PID tuning coefficients
- modified frontside autonomous return to avoid descoring a pixel placed on spike mark by allied team
- frontside autonomous uses some "fancy splines" for movement
- backstage autonomous avoids the pixel placed by self while parking using some more "fancy splines"
- all autonomous now parks on the tile closest to center of backstage to avoid obstructing teammate
 
### Fixed
- fixed frontside autonomous not properly maintaining heading on moves


## [2024-02-01]
 
got tired of resetting the robot manually each time, now the arm resets itself so all we have to do is place the robot to test it again. my favorite feature is the newly added "jiggle" where our robot "calmly" "jiggles" to fully drop a pixel so that when we are placing a pixel on the backdrop, it won't get stuck in the mandibles. We also completely scrapped the arm tilt because it was unreliable and constantly broke itself an dmany things around it.

### Added
- new program for robot reset for faster testing
- added teleop arm safety button functionality
- teleop has a little "jiggle" to help the robot get rid of stuck pixels when placing on backdrop
- added teleop and autonomous initialization move to stay under size constraints
- added telemetry smiley face to indicate that the robot is finished initializing
 
### Changed
- removed arm tilt do to unreliability issues
- modified blue and red front side autonomous move positions for greater accuracy
- robot holds on to the yellow pixel to drop in backstage area


## [2024-01-22]
 
Robot works with the new arm we put on. added toggles because we ran out of buttons on the controller.

### Added
- full toggle button support for teleop
 
### Changed
- config works with new arm
- teleop works with new arm
- autonomous-front works with new arm


## [2024-01-17]
 
notes

### Added
- proper start location coordinates for both red and blue side
- full parking to the red-front-side autonomous
- full blue side with pixel on spike mark and park
  
### Changed
- movement uses vector positions for navigation instead of relative movements
- more PID tuning
- detection checks locations and uses the highest cb or cr value for detection, this is better than the old threshold method because it works properly in very bright or dark rooms.
 
### Fixed


## [2024-01-07]
 
red side autonomous for front side of field works, slight vision errors in varied lighting environments

### Added
- red side pixel drop on spike mark for vision-front
 
### Changed
- adjusted location the camera detects prop for better accuracy


## [2024-01-03] 
 
added OpenCV computer vision, I attempted to use tensor flow for object detection however, due to the nature of the team elements this year, tensorFlow using a trained model is incredibly unreliable

### Added
- OpenCV test program that looks at pixels over spike mark and decides where prop is the current implementation uses a threshold value to decide if the location has a red, green, or no prop


## [2023-12-06]
 
added an autonomous for each possible parking location

### Added
- autoRed1 & autoBlue1 parks closest to wall in backdrop
- autoRed2 & autoBlue2 parks in middle tile of backdrop
- autoRed3 & autoBlue3 parks at tile in backdrop closest to center
- drive constants shared across all autonomous programs
 
### Changed
- further roadrunner tuning adjustments
- completely phased out old drive-by encoder functions



## [2023-11-18]
 
mainly roadrunner improvements

### Added
- full functionality for the winch
- start positions calculated properly for autonomous
 
### Changed
- removed outdated code for old config
 
### Fixed
- fixed distances on raodrunner
- fixed mild tuning errors for roadrunner
- set proper PID tuning coefficients


## [2023-11-12]
 
Roadrunner installed and tuned for the robot and some modifications to teleop

### Added
- installed roadrunner packages
- basic autonomous added using roadrunner functions
 
### Changed
- roadrunner tuned to robot
- teleop works with new suspension winch 
 
### Fixed
- Fixed an issue with toggle buttons



## [2023-11-06]
 
Brought the robot to a functioning state for teleop and old autonomous system

### Added
- toggle functionality for buttons
 
### Fixed
- config file updated to work with new robot

