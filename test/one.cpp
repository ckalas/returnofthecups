#include <iostream>
#include <unistd.h>
#include <fstream>

using namespace std;

int main() {
    int x;
    usleep(3000000);
    cout << "30 -15 -23" << endl;
    cout.flush();
    cin >>  x; 
    cout <<  "received " << x << endl;

}