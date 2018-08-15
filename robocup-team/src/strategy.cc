#include "naobehavior.h"
#include "../rvdraw/rvdraw.h"
#include <map>

extern int agentBodyType;

int DEFENSE_MAX_INDEX = 4;
double PASSING_RADIUS = 5.0;
int CHASE_BALL_NUMBER = 3;
double KICKING_DISTANCE = 0.5;
double ATTACK_RADIUS = 3.0;
double TEAMMATE_AVOIDANCE_DISTANCE = 2.0;
double OPPONENT_AVOIDANCE_DISTANCE = 4.0;

/*
 * Real game beaming.
 * Filling params x y angle
 */
void NaoBehavior::beam( double& beamX, double& beamY, double& beamAngle ) {
    beamX = -HALF_FIELD_X + worldModel->getUNum();
    beamY = 0;
    beamAngle = 0;
}

SkillType NaoBehavior::selectSkill() {
    // Select goalie, defense or offense
    if (worldModel->getUNum() == WO_TEAMMATE1) {
      return goalie();
    } else if (worldModel->getUNum() < WO_TEAMMATE1 + DEFENSE_MAX_INDEX) {
      return defense();
    } else {
      return offense();
    }
}

// OFFENSE BEHAVIOR

SkillType NaoBehavior::offense() {
  // Map agent to distance from ball and opponent's goal
  map<double,int> ballDistMap;
  map<double,int> oppGoalDistMap;
  VecPosition myPos = worldModel->getMyPosition();
  VecPosition ourGoal = VecPosition(-HALF_FIELD_X, 0, 0);
  VecPosition oppGoal = VecPosition(HALF_FIELD_X, 0, 0);

  for(int i = WO_TEAMMATE1; i < WO_TEAMMATE1+NUM_AGENTS; ++i) {
      VecPosition temp;
      int playerNum = i - WO_TEAMMATE1 + 1;
      if (worldModel->getUNum() == playerNum) {
          // This is us
          temp = myPos;
      } else {
          WorldObject* teammate = worldModel->getWorldObject(i);
          if (teammate->validPosition) {
              temp = teammate->pos;
          } else {
              continue;
          }
      }
      temp.setZ(0);
      double distanceToBall = temp.getDistanceTo(ball);
      double distanceToOpponetGoal = temp.getDistanceTo(oppGoal);
      ballDistMap[distanceToBall] = playerNum;
      oppGoalDistMap[distanceToOpponetGoal] = playerNum;
  }

  // Determine how close I am to ball
  int myBallRank =0;
  int counter = 0;
  for (std::map<double,int>::iterator it=ballDistMap.begin(); it!=ballDistMap.end(); ++it){
    if (it->second == worldModel->getUNum()) {
      //find which position I am in the map
      myBallRank = counter;
    }
    counter++;
  }

  // Main logic
  if (myBallRank == 0) {
    // I am closest
    if (ballDistMap.begin()->first < KICKING_DISTANCE ) {
      // I have the ball
      if (myPos.getDistanceTo(oppGoal) < PASSING_RADIUS ) {
        // Shoot
        std::cout<<"Shooting\n";
        return kickBall(KICK_FORWARD, oppGoal);
      } else if (oppGoalDistMap.begin()->second != worldModel->getUNum() && myPos.getDistanceTo(worldModel->getWorldObject(oppGoalDistMap.begin()->second)->pos)<PASSING_RADIUS*1.2) {
        // Pass
        std::cout<<"Passing\n";
        return kickBall(KICK_FORWARD, worldModel->getWorldObject(oppGoalDistMap.begin()->second)->pos);
      } else {
        // Dribble
        std::cout<<"Dribbling\n";
        return kickBall(KICK_DRIBBLE, oppGoal);
      }
    } else {
      // Go to the ball
      std::cout<<"Getting closer to ball\n";
      VecPosition target = collisionAvoidance(false /*teammate*/, false/*opponent*/, false/*ball*/, 1/*proximity thresh*/, .5/*collision thresh*/, ball, true/*keepDistance*/);
      return goToTarget(target);
    }
  } else if (myBallRank < CHASE_BALL_NUMBER) {
    // I am within the 3 closest to ball (chase ball)
    VecPosition targetOffset = VecPosition();
    if (myBallRank == 1) {
      targetOffset.setX(-1);
      targetOffset.setY(-1);
    } else {
      targetOffset.setX(-1);
      targetOffset.setY(1);
    }
    //use a larger collision avoidance
    VecPosition target = collisionAvoidance(true /*teammate*/, false/*opponent*/, false/*ball*/, 2/*proximity thresh*/, 1.5/*collision thresh*/, ball+targetOffset, true/*keepDistance*/);
    return goToTarget(target);
  } else {
    // I am far from the ball (potential field)
    VecPosition force = VecPosition();

    // Force for each teammate
    for (int i = WO_TEAMMATE1; i < WO_TEAMMATE1+NUM_AGENTS; ++i) {
        VecPosition temp;
        int playerNum = i - WO_TEAMMATE1 + 1;
        if (worldModel->getUNum() != playerNum) {
            WorldObject* teammate = worldModel->getWorldObject(i);
            if (teammate->validPosition) {
                temp = teammate->pos;
                temp.setZ(0);
                if (myPos.getDistanceTo(temp) < TEAMMATE_AVOIDANCE_DISTANCE) {
                  force += (temp - myPos)*-1;
                }
            }
        }
    }

    // Force for each opponent
    for (int i = WO_OPPONENT1; i < WO_OPPONENT1+NUM_AGENTS; ++i) {
        VecPosition temp;
        WorldObject* opponent = worldModel->getWorldObject(i);
        if (opponent->validPosition) {
            temp = opponent->pos;
            temp.setZ(0);
            if (myPos.getDistanceTo(temp) < OPPONENT_AVOIDANCE_DISTANCE) {
              force += (temp - myPos)*-1.5;
            }
        }
    }

    // Force for the ball
    if (myPos.getDistanceTo(oppGoal) <= ATTACK_RADIUS) {
      force += (oppGoal - myPos)*-2;
    } else {
      force += (oppGoal - myPos)*1;
    }

    if (myPos.getDistanceTo(ball) >= PASSING_RADIUS) {
      force += (ball - myPos)*1.5;
    }
    // return goToTargetRelative(VecPosition(), 0);
    VecPosition move = force.normalize();
    VecPosition target = collisionAvoidance(true /*teammate*/, false/*opponent*/, false/*ball*/, 1/*proximity thresh*/, .5/*collision thresh*/, myPos+move, true/*keepDistance*/);
    return goToTarget(target);
  }
}

// DEFENSE BEHAVIOR

SkillType NaoBehavior::defense()
{
  int numberOfDefender = DEFENSE_MAX_INDEX - 1;
  double defenseArcRadius = 2.5;
  double defenseConeAngle = 100.0;
  VecPosition defenseArcCenter = VecPosition(-HALF_FIELD_X+1.0, 0, 0);

  VecPosition ballPosition = worldModel->getBall();
  VecPosition goalCenter = VecPosition(-HALF_FIELD_X, 0, 0);

  SIM::AngDeg ballPositionAngle = atan2Deg(ballPosition.getY() - goalCenter.getY(), ballPosition.getX() - goalCenter.getX());
  VecPosition target = defenseArcCenter +
      VecPosition(0, defenseArcRadius, 0).rotateAboutZ(-ballPositionAngle).rotateAboutZ((180-defenseConeAngle)/2).rotateAboutZ(100.0/(numberOfDefender-1)*(worldModel->getUNum() - WO_TEAMMATE2));
  target = collisionAvoidance(true /*teammate*/, false/*opponent*/, false/*ball*/, 0.2/*proximity thresh*/, .1/*collision thresh*/, target, true/*keepDistance*/);

  if(me.getDistanceTo(ballPosition) < 1.2){
    return kickBall(KICK_FORWARD, VecPosition(0, 0, 0));
  }
  return goToTarget(target);
}

// GOALIE BEHAVIOR

SkillType NaoBehavior::goalie()
{
  #define GOAL_DEPTH 0.6
  #define GOAL_WIDTH 2.1
  #define GOALIE_ID WO_TEAMMATE1+10

  VecPosition ballPosition = worldModel->getBall();
  VecPosition goalCenter = VecPosition(-HALF_FIELD_X - GOAL_DEPTH, 0, 0);
  int goalieID = GOALIE_ID;
  float goaliePositionX = -HALF_FIELD_X;
  float scale = (ballPosition.getX() - goalCenter.getX())/(goaliePositionX - goalCenter.getX());
  float goaliePositoinY = (ballPosition.getY() - goalCenter.getY())/scale;
  //clampt goaliePositoinY
  if (goaliePositoinY < -GOAL_WIDTH/2) {
    goaliePositoinY = -GOAL_WIDTH/2;
  }
  else if(goaliePositoinY > GOAL_WIDTH/2)
  {
    goaliePositoinY = GOAL_WIDTH/2;
  }

  VecPosition target = VecPosition(goaliePositionX, goaliePositoinY, 0);
  VecPosition localBall = worldModel->g2l(ballPosition);

  SIM::AngDeg localCenterAngle = atan2Deg(localBall.getY(), localBall.getX());
  if(me.getDistanceTo(ballPosition) < .5)
  {
    return kickBall(KICK_FORWARD, VecPosition(-HALF_FIELD_X/2.0, 0, 0));
  }
  else if (me.getDistanceTo(target) < .05 && abs(localCenterAngle) <= 5) {
      // Close enough to desired position and orientation so just stand
      return SKILL_STAND;
  } else if (me.getDistanceTo(target) < .1) {
      // Close to desired position so start turning to face center
      return goToTargetRelative(worldModel->g2l(target), localCenterAngle);
  } else {
      // Move toward target location
      return goToTarget(target);
  }
}
