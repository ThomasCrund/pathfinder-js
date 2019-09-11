// addon.cc
//#include <node/node.h>
#include <node.h>
#include "pathfinder.h"
#include <iostream>
//#include "include/pathfinder.h"

namespace demo {

using v8::Context;
using v8::Function;
using v8::FunctionCallbackInfo;
using v8::Isolate;
using v8::Local;
using v8::NewStringType;
using v8::Null;
using v8::Object;
using v8::String;
using v8::Value;
using v8::Array;

void GenerateTank(const FunctionCallbackInfo<Value>& args) {
  Isolate* isolate = args.GetIsolate();
Local<Context> context = isolate->GetCurrentContext();

  /*
   *
   * Argument format
   * 
   * int length
   * double[] points
   * double timestep
   * double velocity m/s
   * double acceleration m/s/s
   * double jerk m/s/s/s
   * double wheelbaseWidth m
   * 
   */
  //point length
  if (!args[0]->IsInt32()) {
  isolate->ThrowException(v8::Exception::TypeError(
        v8::String::NewFromUtf8(isolate, "Argument 0 (point length) must be a integer")));
  return;
  }

  int POINT_LENGTH = args[0].As<v8::Integer>()->Value();

  if (!args[1]->IsArray()) {
  isolate->ThrowException(v8::Exception::TypeError(
        v8::String::NewFromUtf8(isolate, "Argument 1 (points) must be a array")));
  return;
  }

  Waypoint *points = (Waypoint*)malloc(sizeof(Waypoint) * POINT_LENGTH);

  Local<Array> pointsInJsArray = Local<Array>::Cast(args[1]);

  for(int i = 0; i < POINT_LENGTH; i++) {
    Local<v8::Value> arrayValue = pointsInJsArray->Get(i);

    if (!arrayValue->IsArray()) {
        isolate->ThrowException(v8::Exception::TypeError(
        v8::String::NewFromUtf8(isolate, "Argument 1 (points) must have a array in a array")));
      return;
    }

     Local<Array> array = Local<Array>::Cast(arrayValue);

    Waypoint point = { array->Get(0).As<v8::Number>()->Value(), array->Get(1).As<v8::Number>()->Value(), d2r(array->Get(2).As<v8::Number>()->Value()) };
    points[i] = point;
  }
  

  
  

  if (!args[2]->IsNumber()) {
  isolate->ThrowException(v8::Exception::TypeError(
        v8::String::NewFromUtf8(isolate, "Argument 2 (time step) must be a number")));
  return;
  }

  double timeStep = args[2].As<v8::Number>()->Value();

  if (!args[3]->IsNumber()) {
  isolate->ThrowException(v8::Exception::TypeError(
        v8::String::NewFromUtf8(isolate, "Argument 3 (velocity) must be a number")));
  return;
  }

  double velocity = args[3].As<v8::Number>()->Value();

  if (!args[4]->IsNumber()) {
  isolate->ThrowException(v8::Exception::TypeError(
        v8::String::NewFromUtf8(isolate, "Argument 4 (acceleration) must be a number")));
  return;
  }

  double acceleration = args[4].As<v8::Number>()->Value();

  if (!args[5]->IsNumber()) {
  isolate->ThrowException(v8::Exception::TypeError(
        v8::String::NewFromUtf8(isolate, "Argument 5 (jerk) must be a number")));
  return;
  }

  double jerk = args[5].As<v8::Number>()->Value();

  if (!args[6]->IsNumber()) {
    isolate->ThrowException(v8::Exception::TypeError(
          v8::String::NewFromUtf8(isolate, "Argument 6 (wheelbase) must be a number")));
    return;
  }

  double wheelbase_width = args[6].As<v8::Number>()->Value();

  try {
    TrajectoryCandidate candidate;
    pathfinder_prepare(points, POINT_LENGTH, FIT_HERMITE_CUBIC, PATHFINDER_SAMPLES_HIGH, timeStep, velocity, acceleration, jerk, &candidate);
    free(points);

    int length = candidate.length;
    std::cout << "Length: " << length << "\n";
    if (length < 0) {
      Local<Function> cb = Local<Function>::Cast(args[8]);
      const unsigned argc = 1;
      Local<Value> argv[argc] = {
          v8::String::NewFromUtf8(isolate, "Could Not Compute Path", NewStringType::kNormal).ToLocalChecked()
      };
      cb->Call(context, Null(isolate), argc, argv).ToLocalChecked();  
      return;
    }
    Segment *trajectory = (Segment*)malloc(length * sizeof(Segment));
    
    pathfinder_generate(&candidate, trajectory);

    Segment *leftTrajectory = (Segment*)malloc(sizeof(Segment) * length);
    Segment *rightTrajectory = (Segment*)malloc(sizeof(Segment) * length);
    
    

    pathfinder_modify_tank(trajectory, length, leftTrajectory, rightTrajectory, wheelbase_width);

    // Do something with the trajectories...

    v8::Local<v8::Array> trajectoryCenter = v8::Array::New(isolate, length);
    v8::Local<v8::Array> trajectoryLeft = v8::Array::New(isolate, length);
    v8::Local<v8::Array> trajectoryRight = v8::Array::New(isolate, length);

    for(int i = 0; i < length; i++) {
      v8::Local<v8::Object> leftObject = v8::Object::New(isolate);
      v8::Local<v8::Object> rightObject = v8::Object::New(isolate);
      v8::Local<v8::Object> centerObject = v8::Object::New(isolate);
      Segment smiddle = trajectory[i];
      Segment sleft = leftTrajectory[i];
      Segment sright = rightTrajectory[i];

      //center profile object set
      centerObject->Set( v8::String::NewFromUtf8(isolate, "x", NewStringType::kNormal).ToLocalChecked(), v8::Number::New(isolate, smiddle.x));
      centerObject->Set( v8::String::NewFromUtf8(isolate, "y", NewStringType::kNormal).ToLocalChecked(), v8::Number::New(isolate, smiddle.y));
      centerObject->Set( v8::String::NewFromUtf8(isolate, "timeStep", NewStringType::kNormal).ToLocalChecked(), v8::Number::New(isolate, smiddle.dt));
      centerObject->Set( v8::String::NewFromUtf8(isolate, "distance", NewStringType::kNormal).ToLocalChecked(), v8::Number::New(isolate, smiddle.position));
      centerObject->Set( v8::String::NewFromUtf8(isolate, "velocity", NewStringType::kNormal).ToLocalChecked(), v8::Number::New(isolate, smiddle.velocity));
      centerObject->Set( v8::String::NewFromUtf8(isolate, "heading", NewStringType::kNormal).ToLocalChecked(), v8::Number::New(isolate, smiddle.heading));
      centerObject->Set( v8::String::NewFromUtf8(isolate, "acceleration", NewStringType::kNormal).ToLocalChecked(), v8::Number::New(isolate, smiddle.acceleration));
      centerObject->Set( v8::String::NewFromUtf8(isolate, "jerk", NewStringType::kNormal).ToLocalChecked(), v8::Number::New(isolate, smiddle.jerk));
      
      //center profile object set
      leftObject->Set( v8::String::NewFromUtf8(isolate, "x", NewStringType::kNormal).ToLocalChecked(), v8::Number::New(isolate, sleft.x));
      leftObject->Set( v8::String::NewFromUtf8(isolate, "y", NewStringType::kNormal).ToLocalChecked(), v8::Number::New(isolate, sleft.y));
      leftObject->Set( v8::String::NewFromUtf8(isolate, "timeStep", NewStringType::kNormal).ToLocalChecked(), v8::Number::New(isolate, sleft.dt));
      leftObject->Set( v8::String::NewFromUtf8(isolate, "distance", NewStringType::kNormal).ToLocalChecked(), v8::Number::New(isolate, sleft.position));
      leftObject->Set( v8::String::NewFromUtf8(isolate, "velocity", NewStringType::kNormal).ToLocalChecked(), v8::Number::New(isolate, sleft.velocity));
      leftObject->Set( v8::String::NewFromUtf8(isolate, "heading", NewStringType::kNormal).ToLocalChecked(), v8::Number::New(isolate, sleft.heading));
      leftObject->Set( v8::String::NewFromUtf8(isolate, "acceleration", NewStringType::kNormal).ToLocalChecked(), v8::Number::New(isolate, sleft.acceleration));
      leftObject->Set( v8::String::NewFromUtf8(isolate, "jerk", NewStringType::kNormal).ToLocalChecked(), v8::Number::New(isolate, sleft.jerk));
      
      //center profile object set
      rightObject->Set( v8::String::NewFromUtf8(isolate, "x", NewStringType::kNormal).ToLocalChecked(), v8::Number::New(isolate, sright.x));
      rightObject->Set( v8::String::NewFromUtf8(isolate, "y", NewStringType::kNormal).ToLocalChecked(), v8::Number::New(isolate, sright.y));
      rightObject->Set( v8::String::NewFromUtf8(isolate, "timeStep", NewStringType::kNormal).ToLocalChecked(), v8::Number::New(isolate, sright.dt));
      rightObject->Set( v8::String::NewFromUtf8(isolate, "distance", NewStringType::kNormal).ToLocalChecked(), v8::Number::New(isolate, sright.position));
      rightObject->Set( v8::String::NewFromUtf8(isolate, "velocity", NewStringType::kNormal).ToLocalChecked(), v8::Number::New(isolate, sright.velocity));
      rightObject->Set( v8::String::NewFromUtf8(isolate, "heading", NewStringType::kNormal).ToLocalChecked(), v8::Number::New(isolate, sright.heading));
      rightObject->Set( v8::String::NewFromUtf8(isolate, "acceleration", NewStringType::kNormal).ToLocalChecked(), v8::Number::New(isolate, sright.acceleration));
      rightObject->Set( v8::String::NewFromUtf8(isolate, "jerk", NewStringType::kNormal).ToLocalChecked(), v8::Number::New(isolate, sright.jerk));
      
      //update array with this spot
      trajectoryCenter->Set(i, centerObject);
      trajectoryLeft->Set(i, leftObject);
      trajectoryRight->Set(i, rightObject);
    }
  
    free(trajectory);
    free(leftTrajectory);
    free(rightTrajectory);

    Local<Function> cb = Local<Function>::Cast(args[7]);
    const unsigned argc = 4;
    Local<Value> argv[argc] = {
        v8::Integer::New(isolate, length),
        trajectoryCenter,
        trajectoryLeft,
        trajectoryRight
    };
    cb->Call(context, Null(isolate), argc, argv).ToLocalChecked();
  } catch(const std::exception& e) {
    Local<Function> cb = Local<Function>::Cast(args[8]);
    const unsigned argc = 1;
    Local<Value> argv[argc] = {
        v8::String::NewFromUtf8(isolate, e.what(), NewStringType::kNormal).ToLocalChecked()
    };
    cb->Call(context, Null(isolate), argc, argv).ToLocalChecked();  
  }
}



// Initialize this addon to be context-aware.
NODE_MODULE_INIT(/* exports, module, context */) {
  Isolate* isolate = context->GetIsolate();

  // Wrap the data in a v8::External so we can pass it to the method we expose.
  //Local<v8::External> external = v8::External::New(isolate, data);

  // Expose the method "Method" to JavaScript, and make sure it receives the
  // per-addon-instance data we created above by passing `external` as the
  // third parameter to the FunctionTemplate constructor.
  //NODE_SET_METHOD(exports, "generateTank", GenerateTank);
  exports->Set(context,
               v8::String::NewFromUtf8(isolate, "generateTank", NewStringType::kNormal)
                  .ToLocalChecked(),
               v8::FunctionTemplate::New(isolate, GenerateTank)
                  ->GetFunction(context).ToLocalChecked()).FromJust();
}

/*void Init(Local<Object> exports, Local<Object> module) {
  NODE_SET_METHOD(exports, "generateTank", GenerateTank);
}

NODE_MODULE(NODE_GYP_MODULE_NAME, Init)
*/
}  