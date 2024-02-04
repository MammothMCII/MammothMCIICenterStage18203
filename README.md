# Mammoth MCII 18203 Center stage





## Basic operation
### Teleop main is the only teleop used in competition grips servos on initialization

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
a --> b[Program Start]
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

rr3 & rl3 --> f[position at backdrop]
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

### Vision Back:
this uses the white tape on the robot to line up with the inner edge of the left side of the tile on the side near the backdrop. 
it follows the basic line 
- check three locations for team prop
- the spot with either the most red or most blue is selected as the location for the team prop
- travels to the line and drops pixel
- then depending on the identified side, parks at the backdrop
