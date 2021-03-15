#include <iostream> // a header file library that lets us work with input and output objects, such as cout
using namespace std; // means that we can use names for objects and variables from the standard library.

int main() {
    cout << "Hello World!" << endl; // << = insertion operator, cout = see-out, << endl = indsætter et linebreak, ligesom "\n" ville have gjort
    // Ovenstående kan skrives som "std::cout << "Hello World!";"", hvor "using namespace std" så kan udelades
    cout << "Next line under first line" << endl;

    // Declaring "identifiers" - i c++ er navnene man giver til variabler "identifiers"
    int a;
    a = 70000;
    cout << "Int 'a' is: " << a << ". this is the end of this line";

    // Declaring many variables
    float b = 13.2, c = 30;
    cout << endl << "b+c is equal to: " << b+c;
    
    // User input
    int x;
    cin >> x;
    cout << "Input is: " << x;


  return 0;
}