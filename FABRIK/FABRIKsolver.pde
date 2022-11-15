//FK & IK 
//CSCI 5611 3-link IK Chain follows mouse [Example]
// Stephen J. Guy <sjguy@umn.edu>

void setup(){
  size(840,680);
  surface.setTitle("Inverse Kinematics [CSCI 5611 Example]");
  strokeWeight(2);
}


//Root
Vec2 root = new Vec2(400, 350);

// Arm 1

//Link 1
float l0 = 100;
float a0 = 0.4;

//Link 2
float l1 = 100;
float a1 = 0.0; 

//Link 3
float l2 = 100;
float a2 = 0.7; 

//Link 4
float l3 = 100;
float a3 = 0.7; 

//Link 5
float l4 = 100;
float a4 = 0.7; 

// Arm 2

//Link 1
float l0_arm2 = 100;
float a0_arm2 = 0.4;

//Link 2
float l1_arm2 = 100;
float a1_arm2 = 0.0; 

//Link 3
float l2_arm2 = 100;
float a2_arm2 = 0.7; 

//Link 4
float l3_arm2 = 100;
float a3_arm2 = 0.7; 

//Link 5
float l4_arm2 = 100;
float a4_arm2 = 0.7; 

Vec2 start_l1, start_l2, start_l3, start_l4, endPoint;

int numJoints = 5;
Vec2 goal = new Vec2(100, 100);
Vec2[] joints = {new Vec2(400, 350), new Vec2(300, 350), new Vec2(200, 350), new Vec2(100, 350), new Vec2(0, 350)};
int[] dists = {100, 100, 100, 100};
float tolerance = 0.1;

Vec2 constrain(Vec2 calc, Vec2 line) {
  float scalar = dot(calc, line) / line.length(); //<>//
  Vec2 proj = (line.normalized()).times(scalar);
  
  float left = -(proj.length() * tan(HALF_PI));
  float right = (proj.length() * tan(HALF_PI));
  float up = (proj.length() * tan(HALF_PI));
  float down = -(proj.length() * tan(HALF_PI));
  
  float xbound = calc.x >=0 ? right : left;
  float ybound = calc.y >=0 ? up : down;
  float ellipse = sq(calc.x)/sq(xbound) + sq(calc.y)/sq(ybound);
  boolean inbounds = ellipse <= 1 && scalar >= 0;
  Vec2 res = calc;
  if (!inbounds) {
    float a = atan2(calc.y, calc.y);
    res.x = xbound * cos(a);
    res.y = ybound * sin(a);
  }
  return res;
}

void backward() {
  joints[numJoints - 1] = goal;
  for (int i = numJoints - 2; i >= 0; i--) { //<>//
    Vec2 r = (joints[i+1].minus(joints[i]));
    float l = dists[i] / r.length();
    joints[i] = joints[i+1].times(1 - l).plus(joints[i].times(l));
  }
}

void forward() {
  joints[0] = root;
  Vec2 coneVec = (joints[1].minus(joints[0])).normalized();
  for (int i = 0; i < numJoints - 1; i++) {
    Vec2 r = joints[i+1].minus(joints[i]);
    float l = dists[i] / r.length(); //<>//
    Vec2 pos = joints[i].times(1 - l).plus(joints[i+1].times(l));
    Vec2 t  = constrain(pos.minus(joints[i]), coneVec);
    joints[i+1] = joints[i].plus(t);
    //joints[i+1] = joints[i].times(1 - l).plus(joints[i+1].times(l));
    coneVec = joints[i+1].minus(joints[i]);
  }
}

void solve(){
  Vec2 startToGoal; //<>//
  startToGoal = goal.minus(root);
  float totaldist = 0;
  for (int i : dists)
    totaldist += i;
  float dist = startToGoal.length();
  if (dist > totaldist) {
    for (int i = 0; i < numJoints - 1; i++) {
      float r = (goal.minus(joints[i])).length();
      float l = dists[i] / r;
      joints[i+1] =  (joints[i].times(1 - l)).plus(goal.times(l));
    }
  } else {
    int bcount = 0;
    float diff = (joints[numJoints - 1].minus(goal)).length();
    while (diff > tolerance) {
      backward();
      forward();
      diff = (joints[numJoints - 1].minus(goal)).length();
      bcount = bcount++;
      if (bcount > 10) break;
    }
  } //<>// //<>//
}

float armW = 20;
boolean tracking1 = false;
boolean tracking2 = false;
void draw(){
  solve();
  
  if (mousePressed == true) {
    if (abs(mouseX - goal.x) < 20 && abs(mouseY - goal.y) < 20) {
      tracking1 = true;
    }
  } else {
    tracking1 = false;
    tracking2 = false;
  }
  if (tracking1 == true) {
    goal.x = mouseX;
    goal.y = mouseY;
  }
  
  background(255,255,255);
  
  fill(180,20,40); //Root
  pushMatrix();
  translate(root.x,root.y);
  rect(-20,-20,40,40);
  popMatrix();
  
  fill(10,150,40); //Green IK Chain
  pushMatrix();
  translate(joints[0].x,joints[0].y);
  circle(0,0,20);
  popMatrix();
  line(joints[0].x, joints[0].y, joints[1].x, joints[1].y);
  line(joints[1].x, joints[1].y, joints[2].x, joints[2].y);
  line(joints[2].x, joints[2].y, joints[3].x, joints[3].y);
  line(joints[3].x, joints[3].y, joints[4].x, joints[4].y);

  
  pushMatrix();
  translate(joints[1].x,joints[1].y);
  circle(0,0,20);
  popMatrix();
  
  pushMatrix();
  translate(joints[2].x,joints[2].y);
  circle(0,0,20);
  popMatrix();
  
  pushMatrix();
  translate(joints[3].x,joints[3].y);
  circle(0,0,20);
  popMatrix();
  
  pushMatrix();
  translate(joints[4].x,joints[4].y);
  circle(0,0,20);
  popMatrix();
  
  fill(0,0,0); //Goal/mouse
  pushMatrix();
  translate(goal.x,goal.y);
  circle(0,0,20);
  popMatrix();
  
  //saveFrame("frame####.png");
}




// Vector Library

public class Vec2 {
  public float x, y;
  
  public Vec2(float x, float y){
    this.x = x;
    this.y = y;
  }
  
  public String toString(){
    return "(" + x+ "," + y +")";
  }
  
  public float length(){
    return sqrt(x*x+y*y);
  }
  
  public Vec2 plus(Vec2 rhs){
    return new Vec2(x+rhs.x, y+rhs.y);
  }
  
  public void add(Vec2 rhs){
    x += rhs.x;
    y += rhs.y;
  }
  
  public Vec2 minus(Vec2 rhs){
    return new Vec2(x-rhs.x, y-rhs.y);
  }
  
  public void subtract(Vec2 rhs){
    x -= rhs.x;
    y -= rhs.y;
  }
  
  public Vec2 times(float rhs){
    return new Vec2(x*rhs, y*rhs);
  }
  
  public void mul(float rhs){
    x *= rhs;
    y *= rhs;
  }
  
  public void clampToLength(float maxL){
    float magnitude = sqrt(x*x + y*y);
    if (magnitude > maxL){
      x *= maxL/magnitude;
      y *= maxL/magnitude;
    }
  }
  
  public void setToLength(float newL){
    float magnitude = sqrt(x*x + y*y);
    x *= newL/magnitude;
    y *= newL/magnitude;
  }
  
  public void normalize(){
    float magnitude = sqrt(x*x + y*y);
    x /= magnitude;
    y /= magnitude;
  }
  
  public Vec2 normalized(){
    float magnitude = sqrt(x*x + y*y);
    return new Vec2(x/magnitude, y/magnitude);
  }
  
  public float distanceTo(Vec2 rhs){
    float dx = rhs.x - x;
    float dy = rhs.y - y;
    return sqrt(dx*dx + dy*dy);
  }
}

Vec2 interpolate(Vec2 a, Vec2 b, float t){
  return a.plus((b.minus(a)).times(t));
}

float interpolate(float a, float b, float t){
  return a + ((b-a)*t);
}

float dot(Vec2 a, Vec2 b){
  return a.x*b.x + a.y*b.y;
}

float cross(Vec2 a, Vec2 b){
  return a.x*b.y - a.y*b.x;
}


Vec2 projAB(Vec2 a, Vec2 b){
  return b.times(a.x*b.x + a.y*b.y);
}

float clamp(float f, float min, float max){
  if (f < min) return min;
  if (f > max) return max;
  return f;
}
