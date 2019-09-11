// test.js
const pathFinder = require('../index.js');

pathFinder.generateTank(3,[[1,1,0],[2,1,0],[3,2,90]],0.02,4.0,3.0,5.0,0.7,(length,cntrTraj,leftTraj,rghtTraj) => {
  console.log(length);
  //console.log(cntrTraj[0]);
  //console.log(cntrTraj);
  console.log(leftTraj[100]);
  //console.log(rghtTraj);
}, (err) => {
  console.log(err);
});
//module.exports = pathFinder;