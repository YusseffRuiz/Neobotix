
"use strict";

let SetMaxVel = require('./SetMaxVel.js')
let SetInt = require('./SetInt.js')
let SetJointStiffness = require('./SetJointStiffness.js')
let SetDefaultVel = require('./SetDefaultVel.js')
let Trigger = require('./Trigger.js')
let SetFloat = require('./SetFloat.js')
let SetJointTrajectory = require('./SetJointTrajectory.js')
let SetOperationMode = require('./SetOperationMode.js')
let GetPoseStampedTransformed = require('./GetPoseStampedTransformed.js')
let SetString = require('./SetString.js')

module.exports = {
  SetMaxVel: SetMaxVel,
  SetInt: SetInt,
  SetJointStiffness: SetJointStiffness,
  SetDefaultVel: SetDefaultVel,
  Trigger: Trigger,
  SetFloat: SetFloat,
  SetJointTrajectory: SetJointTrajectory,
  SetOperationMode: SetOperationMode,
  GetPoseStampedTransformed: GetPoseStampedTransformed,
  SetString: SetString,
};
