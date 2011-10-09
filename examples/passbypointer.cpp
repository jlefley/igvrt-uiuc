#include <iostream>

using namespace std;

void add(int, int);

int main()
{
int a = 1;
int b = 2;

add(a,b);

cout << "a= " << a << " " << "b= " << b << endl;

return 0;
}

void add(int a, int b)
{
a = a + b;
}
