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

void fk(){
  start_l1 = new Vec2(cos(a0)*l0,sin(a0)*l0).plus(root);
  start_l2 = new Vec2(cos(a0+a1)*l1,sin(a0+a1)*l1).plus(start_l1);
  start_l3 = new Vec2(cos(a0+a1+a2)*l2,sin(a0+a1+a2)*l2).plus(start_l2);
  start_l4 = new Vec2(cos(a0+a1+a2+a3)*l3,sin(a0+a1+a2+a3)*l3).plus(start_l3);
  endPoint = new Vec2(cos(a0+a1+a2+a3+a4)*l4,sin(a0+a1+a2+a3+a4)*l4).plus(start_l4);
}

Vec2 start_l1_arm2, start_l2_arm2, start_l3_arm2, start_l4_arm2, endPoint_arm2;
void fk_arm2(){
  start_l1_arm2 = new Vec2(cos(a0_arm2)*l0_arm2,sin(a0_arm2)*l0_arm2).plus(root);
  start_l2_arm2 = new Vec2(cos(a0_arm2+a1_arm2)*l1_arm2,sin(a0_arm2+a1_arm2)*l1_arm2).plus(start_l1_arm2);
  start_l3_arm2 = new Vec2(cos(a0_arm2+a1_arm2+a2_arm2)*l2_arm2,sin(a0_arm2+a1_arm2+a2_arm2)*l2_arm2).plus(start_l2_arm2);
  start_l4_arm2 = new Vec2(cos(a0_arm2+a1_arm2+a2_arm2+a3_arm2)*l3_arm2,sin(a0_arm2+a1_arm2+a2_arm2+a3_arm2)*l3_arm2).plus(start_l3_arm2);
  endPoint_arm2 = new Vec2(cos(a0_arm2+a1_arm2+a2_arm2+a3_arm2+a4_arm2)*l4_arm2,sin(a0_arm2+a1_arm2+a2_arm2+a3_arm2+a4_arm2)*l4_arm2).plus(start_l4_arm2);
}

Vec2 goal1 = new Vec2(100, 100);
Vec2 goal2 = new Vec2(600, 600);
void solve(){
  Vec2 startToGoal, startToEndEffector;
  Vec2 startToGoal_arm2, startToEndEffector_arm2;
  float dotProd; //<>//
    
  startToGoal = goal1.minus(start_l4);
  startToEndEffector = endPoint.minus(start_l4);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  if (cross(startToGoal,startToEndEffector) < 0)
    a4 += acos(dotProd);
  else
    a4 -= acos(dotProd);
  a4 = constrain(a4, -QUARTER_PI, QUARTER_PI);
  fk(); //Update link positions with fk (e.g. end effector changed)
  
  startToGoal = goal1.minus(start_l3);
  startToEndEffector = endPoint.minus(start_l3);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  if (cross(startToGoal,startToEndEffector) < 0)
    a3 += acos(dotProd);
  else
    a3 -= acos(dotProd);
  a3 = constrain(a3, -QUARTER_PI, QUARTER_PI);
  fk(); //Update link positions with fk (e.g. end effector changed)
  
  startToGoal = goal1.minus(start_l2);
  startToEndEffector = endPoint.minus(start_l2);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  if (cross(startToGoal,startToEndEffector) < 0)
    a2 += acos(dotProd);
  else
    a2 -= acos(dotProd);
  a2 = constrain(a2, -QUARTER_PI, QUARTER_PI);
  fk(); //Update link positions with fk (e.g. end effector changed)
  
  startToGoal = goal1.minus(start_l1);
  startToEndEffector = endPoint.minus(start_l1);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  if (cross(startToGoal,startToEndEffector) < 0)
    a1 += acos(dotProd);
  else
    a1 -= acos(dotProd);
  a1 = constrain(a1, -QUARTER_PI, QUARTER_PI);
  fk(); //Update link positions with fk (e.g. end effector changed)
  
  //[CODE FOR LAST LINK GOES HERE]
  startToGoal = goal1.minus(root);
  startToEndEffector = endPoint.minus(root);
  dotProd = dot(startToGoal.normalized(),startToEndEffector.normalized());
  dotProd = clamp(dotProd,-1,1);
  if (cross(startToGoal,startToEndEffector) < 0)
    a0 += acos(dotProd);
  else
    a0 -= acos(dotProd);
  //a0 = constrain(a0, -HALF_PI, HALF_PI);
  fk(); //Update link positions with fk (e.g. end effector changed)
  
  // Arm 2
  startToGoal_arm2 = goal2.minus(start_l4_arm2);
  startToEndEffector_arm2 = endPoint_arm2.minus(start_l4_arm2);
  dotProd = dot(startToGoal_arm2.normalized(),startToEndEffector_arm2.normalized());
  dotProd = clamp(dotProd,-1,1);
  if (cross(startToGoal_arm2,startToEndEffector_arm2) < 0)
    a4_arm2 += acos(dotProd);
  else
    a4_arm2 -= acos(dotProd);
  fk_arm2(); //Update link positions with fk (e.g. end effector changed)
  
  startToGoal_arm2 = goal2.minus(start_l3_arm2);
  startToEndEffector_arm2 = endPoint_arm2.minus(start_l3_arm2); //<>//
  dotProd = dot(startToGoal_arm2.normalized(),startToEndEffector_arm2.normalized());
  dotProd = clamp(dotProd,-1,1);
  if (cross(startToGoal_arm2,startToEndEffector_arm2) < 0) //<>//
    a3_arm2 += acos(dotProd);
  else
    a3_arm2 -= acos(dotProd);
  fk_arm2(); //Update link positions with fk (e.g. end effector changed)
  
  startToGoal_arm2 = goal2.minus(start_l2_arm2);
  startToEndEffector_arm2 = endPoint_arm2.minus(start_l2_arm2);
  dotProd = dot(startToGoal_arm2.normalized(),startToEndEffector_arm2.normalized());
  dotProd = clamp(dotProd,-1,1);
  if (cross(startToGoal_arm2,startToEndEffector_arm2) < 0)
    a2_arm2 += acos(dotProd);
  else
    a2_arm2 -= acos(dotProd);
  fk_arm2(); //Update link positions with fk (e.g. end effector changed)
  
  startToGoal_arm2 = goal2.minus(start_l1_arm2);
  startToEndEffector_arm2 = endPoint_arm2.minus(start_l1_arm2);
  dotProd = dot(startToGoal_arm2.normalized(),startToEndEffector_arm2.normalized());
  dotProd = clamp(dotProd,-1,1);
  if (cross(startToGoal_arm2,startToEndEffector_arm2) < 0)
    a1_arm2 += acos(dotProd);
  else
    a1_arm2 -= acos(dotProd);
  fk_arm2(); //Update link positions with fk (e.g. end effector changed)
  
  //[CODE FOR LAST LINK GOES HERE]
  startToGoal_arm2 = goal2.minus(root);
  startToEndEffector_arm2 = endPoint_arm2.minus(root);
  dotProd = dot(startToGoal_arm2.normalized(),startToEndEffector_arm2.normalized());
  dotProd = clamp(dotProd,-1,1);
  if (cross(startToGoal_arm2,startToEndEffector_arm2) < 0)
    a0_arm2 += acos(dotProd);
  else
    a0_arm2 -= acos(dotProd);
  fk_arm2(); //Update link positions with fk (e.g. end effector changed)
  
  println("Angle 0:",nf(a0,1,2),"Angle 1:",nf(a1,1,2),"Angle 2:",nf(a2,1,2),"Angle 3:",nf(a3,1,2),"Angle 4:",nf(a4,1,2));
}

float armW = 20;
boolean tracking1 = false;
boolean tracking2 = false;
void draw(){
  fk();
  fk_arm2();
  solve();
  
  if (mousePressed == true) {
    if (abs(mouseX - goal1.x) < 20 && abs(mouseY - goal1.y) < 20) {
      tracking1 = true;
    }
    if (abs(mouseX - goal2.x) < 20 && abs(mouseY - goal2.y) < 20) {
      tracking2 = true;
    }
  } else {
    tracking1 = false;
    tracking2 = false;
  }
  if (tracking1 == true) {
    goal1.x = mouseX;
    goal1.y = mouseY;
  }
  if (tracking2 == true) {
    goal2.x = mouseX;
    goal2.y = mouseY;
  }
  
  background(255,255,255);
  
  fill(180,20,40); //Root
  pushMatrix();
  translate(root.x,root.y);
  rect(-20,-20,40,40);
  popMatrix();
  
  
  fill(10,150,40); //Green IK Chain
  pushMatrix();
  translate(root.x,root.y);
  rotate(a0);
  quad(0, -armW/2, l0, -.1*armW, l0, .1*armW, 0, armW/2);
  popMatrix();
  
  pushMatrix();
  translate(start_l1.x,start_l1.y);
  rotate(a0+a1);
  quad(0, -armW/2, l1, -.1*armW, l1, .1*armW, 0, armW/2);
  popMatrix();
  
  pushMatrix();
  translate(start_l2.x,start_l2.y);
  rotate(a0+a1+a2);
  quad(0, -armW/2, l2, -.1*armW, l2, .1*armW, 0, armW/2);
  popMatrix();
  
  pushMatrix();
  translate(start_l3.x,start_l3.y);
  rotate(a0+a1+a2+a3);
  quad(0, -armW/2, l3, -.1*armW, l3, .1*armW, 0, armW/2);
  popMatrix();
  
  pushMatrix();
  translate(start_l4.x,start_l4.y);
  rotate(a0+a1+a2+a3+a4);
  quad(0, -armW/2, l4, -.1*armW, l4, .1*armW, 0, armW/2);
  popMatrix();
  
  // Arm 2
  fill(10,150,40); //Green IK Chain
  pushMatrix();
  translate(root.x,root.y);
  rotate(a0_arm2);
  quad(0, -armW/2, l0, -.1*armW, l0, .1*armW, 0, armW/2);
  popMatrix();
  
  pushMatrix();
  translate(start_l1_arm2.x,start_l1_arm2.y);
  rotate(a0_arm2+a1_arm2);
  quad(0, -armW/2, l1, -.1*armW, l1, .1*armW, 0, armW/2);
  popMatrix();
  
  pushMatrix();
  translate(start_l2_arm2.x,start_l2_arm2.y);
  rotate(a0_arm2+a1_arm2+a2_arm2);
  quad(0, -armW/2, l2, -.1*armW, l2, .1*armW, 0, armW/2);
  popMatrix();
  
  pushMatrix();
  translate(start_l3_arm2.x,start_l3_arm2.y);
  rotate(a0_arm2+a1_arm2+a2_arm2+a3_arm2);
  quad(0, -armW/2, l3, -.1*armW, l3, .1*armW, 0, armW/2);
  popMatrix();
  
  pushMatrix();
  translate(start_l4_arm2.x,start_l4_arm2.y);
  rotate(a0_arm2+a1_arm2+a2_arm2+a3_arm2+a4_arm2);
  quad(0, -armW/2, l4, -.1*armW, l4, .1*armW, 0, armW/2);
  popMatrix();
  
  fill(0,0,0); //Goal/mouse
  pushMatrix();
  translate(goal1.x,goal1.y);
  circle(0,0,20);
  popMatrix();
  
  fill(0,0,0); //Goal/mouse
  pushMatrix();
  translate(goal2.x,goal2.y);
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
