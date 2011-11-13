#include <iostream>

using namespace std;

void add(a, b);

int main()
{
int a = 1;
int b = 2;
int*add_a = &a;
int*add_b = &b;

*add_a = a + b;
//add(a,b);

cout << "a=1" << a << "apple" << "b=2" << b << endl;

return 0;
}

//void add(int a,int b)
//{
//*add_a = a + b;
//}
