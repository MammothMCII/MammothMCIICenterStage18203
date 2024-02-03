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
graph TD
Initialize([Initialize Robot]) --> a[Close Grabber]
a --> b[Program Start]
b --> c{check detection}
c -- Red --> d[set position estimate to red side]
d -- Left --> rl[[run to coodinate]]
d -- Middle --> rm[[run to coodinate]]
d -- Right --> rr[[run to coodinate]]

c -- Blue --> e[set position estimate to blue side]
e -- Left --> bl[[run to coodinate]]
e -- Middle --> bm[[run to coodinate]]
e -- Right --> br[[run to coodinate]]

subgraph Red side
RL([Red Left]) --> rl1[pixel coordinate]
rl1 --> rl2[back coordinate]
rl2 --> rl3[fancy spline]

RM([Red Middle]) --> rm1[pixel coordinate]
rm1 --> rm2[back coordinate]
rm2 --> rm3[fancy spline]

RR([Red Right]) --> rr1[pixel coordinate]
rr1 --> rr2[back coordinate]
rr2 --> rr3[fancy spline]

rr3 & rl3 --> f[position at backdrop]
end

```

### Vision Back:
this uses the white tape on the robot to line up with the inner edge of the left side of the tile on the side near the backdrop. 
it follows the basic line 
- check three locations for team prop
- the spot with either the most red or most blue is selected as the location for the team prop
- travels to the line and drops pixel
- then depending on the identified side, parks at the backdrop
