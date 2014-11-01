#include <iostream>
#include <unistd.h>
#include <fstream>
#include <vector>

using namespace std;

int main() {
    int x;
    cout << 3 << endl << 5 << endl << 100 << endl << 1<< endl;
    cout.flush();
    cin >>  x; 
    cerr <<  "received " << x << endl;

}