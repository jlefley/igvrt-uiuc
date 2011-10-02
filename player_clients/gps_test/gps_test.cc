#include <iostream>
#include <iomanip>
#include <libplayerc++/playerc++.h>
#include <deque>
#include <math.h>

using namespace PlayerCc;
using namespace std;

int main(int argc, char *argv[])
{
	PlayerClient robot("localhost");
	GpsProxy gg(&robot,0);

	while(1)
	{
		robot.Read();

		cout << gg.GetTime() << ": " << setprecision(9) << gg.GetLatitude() << "," << gg.GetLongitude() << endl;
	}
}



