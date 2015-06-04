package org.munoz.indoorassistednavigation.pathplanner;

// TODO: Remove com.googlecode.eyesfree.walkabout 

public class Direction {

	// TODO(jpablomch): Change name of method and class. 
	public static float[] calculateDirectionVector(double[] q) {
	    // Camera vector in camera coordinates.
	    Vector camera = new Vector(0, 0, 1);

	    // Transformation for camera to IMU coordinates.
	    Matrix cameraToIMUTransformation = LinearAlgebraUtils.quaternionToRotationMatrix(
	        new double[] {0, 0, 1, 0});

	    // Transformation for IMU to global coordinates. Need to transpose this one.
	    Matrix imuToGlobalTransformation = LinearAlgebraUtils.quaternionToRotationMatrix(q).transpose();

	    // Do transformation to get the direction the user is facing in global coordinates.
	    Vector user = imuToGlobalTransformation.multiply(cameraToIMUTransformation.multiply(camera));
	    user.normalize();

	    float[] vector = new float[3];
	    for (int i = 0; i < 3; i++) {
	      vector[i] = (float) user.get(i);
	    }

	    return vector;
	}	
}

class LinearAlgebraUtils {

	  public static Matrix quaternionToRotationMatrix(double[] q) {
	    Matrix rotationMatrix = new Matrix();

	    rotationMatrix.set(0, 0, q[0] * q[0] - q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
	    rotationMatrix.set(0, 1, 2 * (q[0] * q[1] + q[2] * q[3]));
	    rotationMatrix.set(0, 2, 2 * (q[0] * q[2] - q[1] * q[3]));

	    rotationMatrix.set(1, 0, 2 * (q[0] * q[1] - q[2] * q[3]));
	    rotationMatrix.set(1, 1, -q[0] * q[0] + q[1] * q[1] - q[2] * q[2] + q[3] * q[3]);
	    rotationMatrix.set(1, 2, 2 * (q[1] * q[2] + q[0] * q[3]));

	    rotationMatrix.set(2, 0, 2 * (q[0] * q[2] + q[1] * q[3]));
	    rotationMatrix.set(2, 1, 2 * (q[1] * q[2] - q[0] * q[3]));
	    rotationMatrix.set(2, 2, -q[0] * q[0] - q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);

	    return rotationMatrix;
	  }

	  

	  
}


/**
 * 3x3 square matrix.
 */
class Matrix {
  public static final int SIZE = 3;
  private final double[][] mData;

  public Matrix() {
    mData = new double[SIZE][SIZE];
  }

  /**
   * Changes value and returns self for chaining.
   */
  public Matrix set(int r, int c, double val) {
    mData[r][c] = val;
    return this;
  }

  public Matrix multiply(Matrix m) {
    Matrix ret = new Matrix();
    for (int r = 0; r < SIZE; r++) {
      for (int c = 0; c < SIZE; c++) {
        Vector v1 = getRow(r);
        Vector v2 = m.getCol(c);
        ret.set(r, c, v1.dot(v2));
      }
    }
    return ret;
  }

  public Vector multiply(Vector v2) {
    Vector ret = new Vector();
    for (int r = 0; r < SIZE; r++) {
      Vector v1 = getRow(r);
      ret.set(r, v1.dot(v2));
    }
    return ret;
  }

  /**
   * Transposes and elements and returns self for chaining.
   */
  public Matrix transpose() {
    double[][] copy = new double[SIZE][SIZE];
    for (int r = 0; r < SIZE; r++) {
      for (int c = 0; c < SIZE; c++) {
        copy[r][c] = mData[r][c];
      }
    }
    for (int r = 0; r < SIZE; r++) {
      for (int c = 0; c < SIZE; c++) {
        mData[r][c] = copy[c][r];
      }
    }
    return this;
  }
  
  /**
   * Gets the row for matrices with dimension 3.
   */
  public Vector getRow(int r) {
    return new Vector(mData[r][0], mData[r][1], mData[r][2]);
  }

  /**
   * Gets the col for matrices with dimension 3.
   */
  public Vector getCol(int c) {
    return new Vector(mData[0][c], mData[1][c], mData[2][c]);
  }

  @Override
  public String toString() {
    return "{" + getRow(0) + "," + getRow(1) + "," + getRow(2) + "}";
  }
}

/**
 * 3x1 vector.
 */
class Vector {
  public static final int X = 0;
  public static final int Y = 1;
  public static final int Z = 2;
  public static final int SIZE = 3;
  private final double[] mData;

  public Vector() {
    mData = new double[SIZE];
  }

  public Vector(double x, double y, double z) {
    mData = new double[SIZE];
    mData[X] = x;
    mData[Y] = y;
    mData[Z] = z;
  }

  /**
   * Changes value and returns self for chaining.
   */
  public Vector set(int i, double val) {
    mData[i] = val;
    return this;
  }

  public double get(int i) {
    return mData[i];
  }

  public double dot(Vector v) {
    double sum = 0;
    for (int i = 0; i < SIZE; i++) {
      sum += mData[i] * v.get(i);
    }
    return sum;
  }

  /**
   * Changes value and returns self for chaining.
   */
  public Vector normalize() {
    final double m = getMagnitude();
    for (int i = 0; i < SIZE; i++) {
      mData[i] /= m;
    }
    return this;
  }

  public Vector cross(Vector v) {
    double x = mData[Y] * v.get(Z) - mData[Z] * v.get(Y);
    double y = mData[Z] * v.get(X) - mData[X] * v.get(Z);
    double z = mData[X] * v.get(Y) - mData[Y] * v.get(X);
    return new Vector(x, y, z);
  }

  public double getMagnitude() {
    double sum = 0;
    for (int i = 0; i < SIZE; i++) {
      sum += mData[i] * mData[i];
    }
    return Math.sqrt(sum);
  }

  @Override
  public String toString() {
    return "{" + mData[0] + "," + mData[1] + "," + mData[2] + "}";
  }
}
