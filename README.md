# Pathfinder JS

A node.js addod of the C++ library [Pathfinder](https://github.com/JacisNonsense/Pathfinder) written by [Jaci](https://github.com/JacisNonsense) 

## Install
```bash
npm i pathfinder1-jaci-js
```
## Examples
* Generate a profile for Tank robot and print the left wheel trajectory
```javascript
var pathLength = 3;
var pathPoints = [[1,1,0],[2,1,0],[3,2,90]];
var timeStep = 0.02;
var maxVelocity = 3;            // m / s
var maxAccelaration = 4;        // m / s / s
var maxJerk = 5;                // m / s / s / s
var wheelBase = 0.7;            // m

pathFinder.generateTank( pathLength, pathPoints, timeStep, maxVelocity, maxAccelaration, maxJerk, wheelBase,(length,centerTrajectory,leftTrajectory,rightTrajectory) => {
  console.log(leftTrajectory);
  //Do something with the trajectory
}, (err) => {
  //Catch Errors in points
});
```