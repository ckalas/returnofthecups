#include <iostream>
#include <unistd.h>
 #include <stdlib.h>
#include <fstream>
#include <vector>

using namespace std;

int main() {
    int x;
    int to[2], from[2];
    pipe(to);
    pipe(from);

    // childs perspective to/from
    if (fork()) { // parent
        close(to[0]);
        close(from[1]);
        dup2(to[1], 1);
        dup2(from[0], 0);
    }
    else {
        close(to[1]);
        close(from[0]);
        dup2(to[0], 0);
        dup2(from[1], 1);
        close(to[0]);
        close(from[1]);
        execvp("two", NULL);
    }
    cout << 32 << endl << 54 << endl << 100 << endl << 1<< endl;
    cout.flush();
    cin >>  x; 
    cerr <<  "received " << x << endl;

}