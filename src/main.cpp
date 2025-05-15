#include <Eigen/Dense>
#include <iostream>

using namespace Eigen;
using namespace std;

int main() {
    // Voorbeeldmatrix A (3x3)
    Matrix3f A;
    A << 1, 2, 3,
         4, 5, 6,
         7, 8, 9;

    // Voorbeeldvector B (3x1)
    Vector3f B;
    B << 1,
         2,
         3;

    // Matrixvermenigvuldiging: C = A * B
    Vector3f C = A * B;

    // Resultaten printen
    cout << "Matrix A:" << endl << A << endl;
    cout << "\nVector B:" << endl << B << endl;
    cout << "\nResultaat van A * B:" << endl << C << endl;

    // Transpositie van A
    Matrix3f A_T = A.transpose();
    cout << "\nTranspositie van A:" << endl << A_T << endl;

    // Inversie (dit faalt omdat A singulier is, ik maak een nieuwe matrix)
    Matrix3f D;
    D << 4, 7, 2,
         3, 6, 1,
         2, 5, 1;

    cout << "\nMatrix D:" << endl << D << endl;
    cout << "\nInversie van D:" << endl << D.inverse() << endl;

    // Determinant van D
    cout << "\nDeterminant van D: " << D.determinant() << endl;

    return 0;
}
