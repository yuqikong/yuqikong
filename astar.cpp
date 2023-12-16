#include <iostream>
#include <stack>
#include <vector>
#include <cmath>
#include <queue>
#include <unordered_set> 
#include <unordered_map> 
 
using namespace std;
 
struct Point
{
	int x,y;
	Point(int x,int y):x(x),y(y){}
	double distance(const Point &p) const
	{
		return sqrt((x-p.x)*(x-p.x)+(y-p.y)*(y-p.y));
	}
	int point2index(int row, int col) const
	{
		return (y + x*col);
	}
};
 
struct Node
{
	Point point;
	double f;
	double g;
	double h;
	Node *parent;
	Node(const Point &point, double g, double h, Node *parent=nullptr):point(point),g(g),h(h),f(g+h),parent(parent){}
};
 
struct my_cmp
{
	// 对priority_queue排序进行自定义，f值小的排在前面
	bool operator()(Node *n1, Node *n2)
	{
		return (n1->f)>(n2->f);
	}
};
 
stack<Point> Astar(const vector<vector<int>> &gridmap, const Point &start, const Point &goal)
{
	int row = gridmap.size();
	int col = gridmap[0].size();
	stack<Point> path;
	Node *s = new Node(start, start.distance(start), start.distance(goal));
	priority_queue<Node *, vector<Node *>, my_cmp> open_list;
	unordered_map<int,Node *> open_dict;
	unordered_set<int> close_dict;
	open_list.push(s);
	open_dict.insert({s->point.point2index(row,col),s});
	while(!open_list.empty())
	{
 
		Node *current = open_list.top();
		open_list.pop();
		open_dict.erase(current->point.point2index(row,col));
		close_dict.insert(current->point.point2index(row,col));
 
		if(current->point.x==goal.x&&current->point.y==goal.y)
		{
			while(current!=nullptr)
			{
				path.push(current->point);
				current=current->parent;
			}
 
			while(!open_list.empty())
			{
				Node *temp = open_list.top();
				open_list.pop();
				delete(temp);
			}
 
			return path;
		}
		int x = current->point.x;
		int y = current->point.y;
		vector<Point> neighbours = {{x-1,y-1},{x-1,y},{x-1,y+1},{x,y-1},{x,y+1},{x+1,y-1},{x+1,y},{x+1,y+1}};
		for(const Point &n:neighbours)
		{
			if(n.x<0||n.x==row||n.y<0||n.y==col||gridmap[n.x][n.y]==1)	continue;
			int index = n.point2index(row,col);
			bool in_close = false;
			if(close_dict.find(index)!=close_dict.end())
			{
				in_close = true;
			}
			if(in_close)	continue;
 
 			double current2n;
			if((abs(current->point.x-n.x) + abs(current->point.y-n.y)) == 1)
				current2n = 1;
			else
				current2n = 1.414;
			double g = current->g + current2n;
			
			bool in_open = false;
			if(open_dict.find(index)!=open_dict.end())
			{
				in_open = true;
 
				if(open_dict[index]->g > g)
				{
					open_dict[index]->g = g;
					open_dict[index]->f = open_dict[index]->h + open_dict[index]->g;
					open_dict[index]->parent = current;
				}
				
			}
			if(in_open)	continue;
 
			double h = n.distance(goal);
			double f = g + h;
			Node * new_n = new Node(n,g,h,current);
			open_list.push(new_n);
			open_dict.insert({n.point2index(row,col),new_n});
 
		}
	}
	while(!open_list.empty())
	{
		Node *temp = open_list.top();
		open_list.pop();
		delete(temp);
	}
	return path;
}
 
int main()
{
    vector<vector<int>>gridmap={
        {0,0,1,0,0},
        {0,1,1,1,0},
        {0,0,0,0,0},
        {1,0,1,0,1},
        {0,0,0,0,0}
    };
    //定义起点和终点
    Point start(0,0);
    Point goal(4,4);
    stack<Point> path=Astar(gridmap,start,goal);
	cout<<"path:";
    while(!path.empty())
    {
        Point temp = path.top();
        path.pop();
        cout<<"("<<temp.x<<","<<temp.y<<") ";
    }
   return 0;
}
