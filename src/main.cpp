#include <Arduino.h>
#include <limits.h>
#include <EEPROM.h>

#define GRID_SIZE 5
#define MAX_PATH_LEN 25

int pathX[MAX_PATH_LEN];
int pathY[MAX_PATH_LEN];
int pathLen = 0;
int pathIndex = 1; // Start from 1, since 0 is the start node
int robotX = 0, robotY = 0;
int heading = 2; // 0=up, 1=right, 2=down, 3=left

struct Node {
  int x, y;
  int g, h, f;
  Node* parent;
  bool walkable;
  int cost;
  char name; // Add name field
};

Node grid[GRID_SIZE][GRID_SIZE];

// Simple Manhattan distance heuristic
int heuristic(int x1, int y1, int x2, int y2) {
  return abs(x1 - x2) + abs(y1 - y2);
}

// Initialize grid (all walkable for now)
void initGrid() {
  char nodeName = 'A';
  // Manually set costs for each node (row by row)
  int manualCosts[GRID_SIZE][GRID_SIZE] = {
    {1, 3, 5, 7, 9},
    {3, 3, 1, 7, 6},
    {5, 1, 2, 3, 4},
    {7, 7, 3, 1, 2},
    {9, 6, 4, 2, 1},
    
  };

  for (int i = 0; i < GRID_SIZE; i++) {
    for (int j = 0; j < GRID_SIZE; j++) {
      int cost = manualCosts[i][j];
      grid[i][j] = {i, j, 0, 0, 0, nullptr, true, cost, nodeName};
      nodeName++;
      // If you want to wrap after 'Z', uncomment the next line:
      // if (nodeName > 'Z') nodeName = 'a';
    }
  }
}

struct ListNode {
  Node* node;
  ListNode* next;
};

// Helper to add node to open list (sorted by f)
void addToOpen(ListNode*& open, Node* node) {
  ListNode* newNode = new ListNode{node, nullptr};
  if (!open || node->f < open->node->f) {
    newNode->next = open;
    open = newNode;
    return;
  }
  ListNode* curr = open;
  while (curr->next && curr->next->node->f <= node->f) curr = curr->next;
  newNode->next = curr->next;
  curr->next = newNode;
}

// Helper to check if node is in list
bool inList(ListNode* list, Node* node) {
  while (list) {
    if (list->node == node) return true;
    list = list->next;
  }
  return false;
}

// Remove and return node with lowest f from open list
Node* popOpen(ListNode*& open) {
  if (!open) return nullptr;
  Node* node = open->node;
  ListNode* temp = open;
  open = open->next;
  delete temp;
  return node;
}

// A* implementation
void astar(int startX, int startY, int goalX, int goalY) {
  // Reset grid node values
  for (int i = 0; i < GRID_SIZE; i++)
    for (int j = 0; j < GRID_SIZE; j++) {
      grid[i][j].g = INT_MAX;
      grid[i][j].h = 0;
      grid[i][j].f = INT_MAX;
      grid[i][j].parent = nullptr;
    }

  Node* start = &grid[startX][startY];
  Node* goal = &grid[goalX][goalY];

  start->g = 0;
  start->h = heuristic(startX, startY, goalX, goalY);
  start->f = start->h;

  ListNode* open = nullptr;
  ListNode* closed = nullptr;
  addToOpen(open, start);

  int dx[4] = {1, -1, 0, 0};
  int dy[4] = {0, 0, 1, -1};

  while (open) {
    Node* current = popOpen(open);
    if (current == goal) {
      // Path found, print it
      Serial.print("Path: ");
      
      pathLen = 0;
      Node* pathNode = goal;
      while (pathNode) {
        pathX[pathLen] = pathNode->x;
        pathY[pathLen] = pathNode->y;
        pathLen++;
        pathNode = pathNode->parent;
      }
      Serial.println();
      
      // Reverse the path arrays so [0] is start, [pathLen-1] is goal
      for (int i = 0; i < pathLen / 2; i++) {
        int tx = pathX[i], ty = pathY[i];
        pathX[i] = pathX[pathLen - 1 - i];
        pathY[i] = pathY[pathLen - 1 - i];
        pathX[pathLen - 1 - i] = tx;
        pathY[pathLen - 1 - i] = ty;
      }
      pathIndex = 1; // Next node to go to (0 is start)
      
      // Clean up
      while (open) popOpen(open);
      while (closed) popOpen(closed);
      return;
    }
    addToOpen(closed, current);

    for (int d = 0; d < 4; d++) {
      int nx = current->x + dx[d];
      int ny = current->y + dy[d];
      if (nx < 0 || ny < 0 || nx >= GRID_SIZE || ny >= GRID_SIZE) continue;
      Node* neighbor = &grid[nx][ny];
      if (!neighbor->walkable || inList(closed, neighbor)) continue;
      int tentative_g = current->g + neighbor->cost;
      if (tentative_g < neighbor->g) {
        neighbor->parent = current;
        neighbor->g = tentative_g;
        neighbor->h = heuristic(nx, ny, goalX, goalY);
        neighbor->f = neighbor->g + neighbor->h;
        if (!inList(open, neighbor)) addToOpen(open, neighbor);
      }
    }
  }
  Serial.println("No path found.");
  // Clean up
  while (open) popOpen(open);
  while (closed) popOpen(closed);
}

// Define named nodes with their coordinates
enum NamedNode { NODE_A, NODE_B, NODE_C, NODE_D, NODE_E, NODE_F, NODE_G, NODE_COUNT };

struct NamedNodeInfo {
  char name;
  int x, y;
};

NamedNodeInfo namedNodes[NODE_COUNT] = {
  {'A', 0, 0}, // Start
  {'B', 0, 2},
  {'C', 1, 2},
  {'D', 2, 2},
  {'E', 3, 2},
  {'F', 4, 2},
  {'G', 4, 4}  // Goal
};

void turnLeft() {
  // Example: left wheel backward, right wheel forward
   digitalWrite(2,LOW);
    analogWrite(5,120);
    digitalWrite(4,LOW);
    analogWrite(6,120);
  delay(350); // Adjust this delay for a 90-degree turn
  // Stop motors after turn
  digitalWrite(2, LOW);
  analogWrite(5, 0);
  digitalWrite(4, LOW);
  analogWrite(6, 0);
}

void turnRight() {
  // Example: left wheel forward, right wheel backward
    digitalWrite(2,HIGH);
    analogWrite(5,120);
    digitalWrite(4,HIGH);
    analogWrite(6,120);
  delay(350); // Adjust this delay for a 90-degree turn
  // Stop motors after turn
  digitalWrite(2, LOW);
  analogWrite(5, 0);
  digitalWrite(4, LOW);
  analogWrite(6, 0);
}

void uTurn() {
  // Example: spin in place 180 degrees
  digitalWrite(2, HIGH);
  analogWrite(5, 80);
  digitalWrite(4, LOW);
  analogWrite(6, 80);
  delay(700); // Adjust for 180-degree turn
  digitalWrite(2, LOW);
  analogWrite(5, 0);
  digitalWrite(4, LOW);
  analogWrite(6, 0);
}

void Infrared_Tracing() {
  int Left_Tra_Value = 1;
  int Center_Tra_Value = 1;
  int Right_Tra_Value = 1;
  int Black = 1;
  Left_Tra_Value = digitalRead(7);
  Center_Tra_Value = digitalRead(8);
  Right_Tra_Value = digitalRead(9);
  if (Left_Tra_Value != Black && (Center_Tra_Value == Black && Right_Tra_Value != Black)) {
    digitalWrite(2,HIGH);
    analogWrite(5,80);   // slower
    digitalWrite(4,LOW);
    analogWrite(6,80);   // slower

  } else if (Left_Tra_Value == Black && (Center_Tra_Value == Black && Right_Tra_Value != Black)) {
    digitalWrite(2,LOW);
    analogWrite(5,50);   // slower
    digitalWrite(4,LOW);
    analogWrite(6,50);   // slower
  } else if (Left_Tra_Value == Black && (Center_Tra_Value != Black && Right_Tra_Value != Black)) {
    digitalWrite(2,LOW);
    analogWrite(5,80);   // slower
    digitalWrite(4,LOW);
    analogWrite(6,80);   // slower
  } else if (Left_Tra_Value != Black && (Center_Tra_Value != Black && Right_Tra_Value == Black)) {
    digitalWrite(2,HIGH);
    analogWrite(5,80);   // slower
    digitalWrite(4,HIGH);
    analogWrite(6,80);   // slower
  } else if (Left_Tra_Value != Black && (Center_Tra_Value == Black && Right_Tra_Value == Black)) {
    digitalWrite(2,HIGH);
    analogWrite(5,50);   // slower
    digitalWrite(4,HIGH);
    analogWrite(6,50);   // slower
  } else if (Left_Tra_Value == Black && Center_Tra_Value == Black && Right_Tra_Value == Black) {
    if (robotX == pathX[pathLen-1] && robotY == pathY[pathLen-1]) {
      // At goal, stop
      digitalWrite(2, LOW);
      analogWrite(5, 0);
      digitalWrite(4, LOW);
      analogWrite(6, 0);
      return;
    }

    int nextX = pathX[pathIndex];
    int nextY = pathY[pathIndex];

    int desiredHeading = heading;
    if (nextX > robotX) desiredHeading = 2;      // down
    else if (nextX < robotX) desiredHeading = 0; // up
    else if (nextY > robotY) desiredHeading = 1; // right
    else if (nextY < robotY) desiredHeading = 3; // left

    int turn = (desiredHeading - heading + 4) % 4;
    if (turn == 1) {
      Serial.println("Turning right");
      turnRight();
    } else if (turn == 3) {
      Serial.println("Turning left");
      turnLeft();
    } else if (turn == 2) {
      Serial.println("U-turn");
      uTurn();
    } else {
      Serial.println("No turn (straight)");
    }
    heading = desiredHeading;

    robotX = nextX;
    robotY = nextY;
    pathIndex++;
    delay(200);
    return;
}
}

void setup(){
  Serial.begin(9600);
  pinMode(7, INPUT);
  pinMode(8, INPUT);
  pinMode(9, INPUT);
  pinMode(2, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(6, OUTPUT);

  initGrid();

  // Use A as start and Y as goal
  int startX = 0;
  int startY = 0;
  int goalX = 4;
  int goalY = 4;

  Serial.print("Start: "); Serial.println(grid[startX][startY].name);
  Serial.print("Goal: "); Serial.println(grid[goalX][goalY].name);

  astar(startX, startY, goalX, goalY);
}

void loop(){
  Infrared_Tracing();

}

