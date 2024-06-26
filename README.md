# Mammoth MCII 18203 Center stage

# WARNING: outdated



## Basic operation
### Teleop main:
this is the only teleop used in competition
the controls are:
- Left Analog y-axis: robot rotation
- Right Analog: robot movement, uses strafe with mecanum wheels
- DPad Up: winch up
- DPad Down: winch down
- DPad Left: release winch*
- DPad Right: launch plane*
- L1(Left Bumper): Top Gripper*
- R1(Right Bumper): Bottom Gripper*
- L2(Left Trigger): Analog speed reduction
- Triangle: Flip hand*
- Square: "jiggle"
- Cross: Arm down
- Circle: Arm up

"*" indicates toggle
![image](https://raw.githubusercontent.com/MammothMCII/MammothMCIICenterStage18203/master/PS4-Controller-diagram-light-mode-freindly.png)

The "jiggle" is where the robot does a little shaking to dislodge any pixels that may be stuck in the gripper when placed on the backdrop. While release winch and launch plane are both on toggles, this is merely for resetting purposes as both of these can only be activated once per game without being manually reset by hand.

### Vision Front:
this uses the white tape on the robot to line up with the inner edge of the left side of the tile on the side away from the backdrop. 
it follows the basic line 
- check three locations for team prop
- the spot with either the most red or most blue is selected as the location for the team prop
- travels to the line and drops pixel
- then depending on the identified side, parks at the backdrop

### Vision Back:
this uses the white tape on the robot to line up with the inner edge of the left side of the tile on the side near the backdrop. 
it follows the basic line 
- check three locations for team prop
- the spot with either the most red or most blue is selected as the location for the team prop
- travels to the line and drops pixel
- then depending on the identified side, parks at the backdrop

this one is logically identical to the front-side autonomous but the coordinates that the robot moves to are changed


#### Full Diagram:
```mermaid
graph LR
Initialize([Initialize Robot]) --> a[Close Grabber]
a --> checkinput[[get pre randomization user input]]
checkinput --> b[\Wait for start/]
b --> c{check detection}

c --> d[set position estimate to red side]
c --> e[set position estimate to blue side]

mtplb & mtprb & mtplr & mtprr--> fin([end])

BL[pixel coordinate] --> bl2[move to wait left]

BM[pixel coordinate] --> bl2[move to wait left]

BR[pixel coordinate] --> bm2[move to wait right]

bl2 & bm2 --> wait[wait for preselected time] --> retRB[return to backdrop] --> lftarm[lift arm]
lftarm --> bdlb[move to backdrop left] & bdmb[move to backdrop middle] & bdrb[move to backdrop right] --> dpx[release yellow pixel] --> wggl
wggl -- park left selected --> mtplb[move to left parking place]
wggl -- park right selected --> mtprb[move to right parking place]

subgraph Blue side
e -- Left --> BL
e -- Middle --> BM
e -- Right -->BR
bl2
bm2
mtplb
mtprb

retRB

subgraph Blue left
BL
bdlb
end
subgraph Blue Middle
BM
bdmb
end
subgraph Blue Right
BR
bdrb
end

end

RL[pixel coordinate] --> rl2[move to wait left]

RM[pixel coordinate] --> rm2[move to wait right]

RR[pixel coordinate] --> rm2[move to wait right]

rl2 & rm2 --> wait[wait for preselected time] --> retRR[return to backdrop] --> lftarm[lift arm]
lftarm --> rdlb[move to backdrop left] & rdmb[move to backdrop middle] & rdrb[move to backdrop right] --> dpx[release yellow pixel] --> wggl
wggl -- park left selected --> mtplr[move to left parking place]
wggl -- park right selected --> mtprr[move to right parking place]

wggl[[wiggle to dislodge]]

subgraph Red side

d -- Left --> RL
d -- Middle --> RM
d -- Right -->RR
rl2
rm2
mtplr
mtprr

retRR

subgraph Red left
RL
rdlb
end
subgraph Red Middle
RM
rdmb
end
subgraph Red Right
RR
rdrb
end
end


subgraph wiggle
wg([wiggle]) --> pw0[set all drive motor power to 1] -- wait 10ms --> pw1[set all drive motor power to -1] -- wait 10ms --> pw2[set all drive motor power to 0] --> kwg{keep wiggleing}
kwg -- yes --> pw0
kwg -- no --> endwg([break])
end
```

during pre-randomization initialization the contolls are as follows:

- DPad Up: set park middle
- DPad Left: set park left
- DPad Right: set park right
- Square: no wait
- Triangle: 3s wait
- Circle: 5s wait
- - Cross: max wait
- Touchpad: accept


In autonomous we discovered that there is one spot where the pixel can reliably be placed from a height and not bounce out of its intended position

