#include "tgaimage.h"
#include <iostream>

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red = TGAColor(255, 0, 0, 255);
const TGAColor blue = TGAColor(0, 0, 255, 255);

void line(int, int, int, int, TGAImage *, TGAColor);

int main(int argc, char **argv) {
  TGAImage image(100, 100, TGAImage::RGB);
  // image.set(52, 41, red);
  line(13, 20, 80, 40, &image, white);
  line(20, 13, 40, 80, &image, red);
  line(80, 40, 13, 20, &image, red);
  image.flip_vertically(); // i want to have the origin at the left bottom
                           // corner of the image
  image.write_tga_file("output.tga");
  return 0;
}

void line(int x0, int y0, int x1, int y1, TGAImage *image, TGAColor color) {
  /**
   * Draws a line using Bresenham's line algorithm
   *
   * The following equation represents a 2D line
   *
   * f(x,y) := Ax + By + C
   * where:
   *    A := delta(y) = y1 - y0
   *    B := - delta(x) = -(x1 - x0)
   *    C := delta(x) * y0  <-- y0 is an arbitrary y-coordinate on the line
   *
   * Start by coloring the initial coordiantes (x0, y0). Assuming a line with a
   * positive slope less than 1 and x0 < x1 and y0 < y1 (i.e. line moves up and
   * to the right), the following can help us determine whether to move our
   * y-coordinate up or remain the same:
   *
   * D := f(x0 + 1, y0 + 0.5)
   *
   * If the value of D > 0, the y-coordinate should increase. Otherwise, the
   * y-coordinate remains the same.
   *
   * To avoid float arithmetic (the equation for D above requires adding 0.5),
   * we can instead work with the difference between points (i.e. look at the
   * delta of D).
   *
   * delta(D) := f(x0 + 1, y0 + 0.5) - f(x0, y0)
   *           = A (x0 +1) + B (y0 + 0.5) + C - A (x0) + B (y0) - C
   *           = A x0 + A + B y0 + 0.5 B + C - A x0 - B y0 - C
   *           = A + 0.5 B
   *           = delta(y) + 0.5 (-delta(x))
   *           = delta(y) - 0.5 delta(x)
   *
   * We don't care about the exact value of delta(D) (or D), just the sign. We
   * can set delta(D) equal to 0 and multiple both sides of the equation by 2 to
   * remove the 0.5 factor (no more floats).
   *
   * delta(D) = 0
   * delta(y) - 0.5 delta(x) = 0
   * 2 delta(y) - delta(x) = 0
   *
   * See: https://en.wikipedia.org/wiki/Bresenham%27s_line_algorithm
   */

  // Will be set if the absolute value of our slope is greather than 1.
  bool steep = false;
  if (std::abs(x1 - x0) < std::abs(y1 - y0)) {
    // Swap x and y values to get a slope with absolute value < 1 (i.e. raise
    // the slope to the negative first power to get the inverse). This is done
    // to make sure the math is correct but we have to make sure to take this
    // into account when drawing the pixels (`steep` helps us keep track of
    // this).
    std::swap(x0, y0);
    std::swap(x1, y1);
    steep = true;
  }

  if (x0 > x1) {
    // Algorithm assumes we move to the right (increasing x-coordinate). Swap
    // our points to guarantee this assumption.
    std::swap(x0, x1);
    std::swap(y0, y1);
  }

  int dx = x1 - x0;
  int dy = y1 - y0;
  int dD = 2 * dy - dx;

  int y = y0;
  for (int x = x0; x <= x1; x++) {
    if (steep) {
      image->set(y, x, color);

    } else {
      image->set(x, y, color);
    }

    // Update delta(D) and update y-coordinate if necessary.
    if (dD > 0) {
      y += 1;
      dD -= 2 * dx;
    }
    dD += 2 * dy;
  }
}
