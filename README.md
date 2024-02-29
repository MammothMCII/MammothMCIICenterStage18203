# Mammoth MCII 18203 Center stage





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
#### Full Diagram:
```mermaid
graph LR
Initialize([Initialize Robot]) --> a[Close Grabber]
a --> checkinput[[get pre randomization user input]]
checkinput --> b[\Wait for start/]
b --> c{check detection}

c --> d[set position estimate to red side]
c --> e[set position estimate to blue side]

subgraph Red side
d -- Left --> RL
d -- Middle --> RM
d -- Right -->RR

RL[pixel coordinate] --> rl2[back coordinate]
rl2 --> rl3[fancy spline]

RM[pixel coordinate] --> rm2[back coordinate]
rm2 --> rm3[fancy spline]

RR[pixel coordinate] --> rr2[back coordinate]
rr2 --> rr3[fancy spline]

rr3 & rl3 --> f[Tile at backdrop closest to center]
end



subgraph Blue side

e -- Left --> BL
e -- Middle --> BM
e -- Right --> BR

BL[pixel coordinate] --> bl3[back coordinate]

BM[pixel coordinate] --> bm3[back coordinate]

BR[pixel coordinate] --> br3[back coordinate]

br3 & bl3 & bm3 --> fb[position at backdrop]

end

Input([get pre randomization user input]) --> chek[\check input/]
chek --> decide{what is the input}
decide -- dpad up --> dup[set end position to middle]
decide -- touchpad --> break
dup --> chek


```

### Vision Back:
this uses the white tape on the robot to line up with the inner edge of the left side of the tile on the side near the backdrop. 
it follows the basic line 
- check three locations for team prop
- the spot with either the most red or most blue is selected as the location for the team prop
- travels to the line and drops pixel
- then depending on the identified side, parks at the backdrop
#### Full Diagram:
```mermaid
graph LR
Initialize([Initialize Robot]) --> a[Close Grabber]
a --> b[Program Start]
b --> c{check detection}

c --> d[set position estimate to red side]
c --> e[set position estimate to blue side]

subgraph Red side
d -- Left --> RL
d -- Middle --> RM
d -- Right -->RR

RL[pixel coordinate] --> rl2[back coordinate]

RM[pixel coordinate] --> rm2[back coordinate]

RR[pixel coordinate] --> rr2[back coordinate]

rr2 & rl2 & rm2 --> f[position at backdrop]
end



subgraph Blue side

e -- Left --> BL
e -- Middle --> BM
e -- Right --> BR

BL[pixel coordinate] --> bl3[back coordinate]

BM[pixel coordinate] --> bm3[back coordinate]

BR[pixel coordinate] --> br3[back coordinate]

br3 & bl3 & bm3 --> fb[position at backdrop]
end

```


In autonomous we discovered that there is one spot where the pixel can reliably be placed from a heigt and not bounce out of its intended position

