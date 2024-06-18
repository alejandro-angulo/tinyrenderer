#include "geometry.h"
#include "model.h"
#include "tgaimage.h"
#include <iostream>

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red = TGAColor(255, 0, 0, 255);
const TGAColor green = TGAColor(0, 255, 0, 255);
const TGAColor blue = TGAColor(0, 0, 255, 255);
const int height = 200;
const int width = 200;

void line(int x0, int y0, int x1, int y1, TGAImage &image, TGAColor color) {
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

  // Used to account for negative slopes (need to decrement y in the for loop
  // below and negate dy to keep the math straight
  int y_step = (y1 > y0) ? 1 : -1;

  int dx = x1 - x0;
  int dy = (y1 - y0) * y_step;
  int dD = 2 * dy - dx;

  int y = y0;
  for (int x = x0; x <= x1; x++) {
    if (steep) {
      image.set(y, x, color);
    } else {
      image.set(x, y, color);
    }

    // Update delta(D) and update y-coordinate if necessary.
    if (dD > 0) {
      y += y_step;
      dD -= 2 * dx;
    }
    dD += 2 * dy;
  }
}

void line(Vec2i v0, Vec2i v1, TGAImage &image, TGAColor color) {
  line(v0.x, v0.y, v1.x, v1.y, image, color);
}

void triangle(Vec2i v0, Vec2i v1, Vec2i v2, TGAImage &image, TGAColor color) {
  /**
   * Draws a filled in triangle
   *
   * First, the vertices are sorted in descending order based on y-coordinates
   * (v0 is the largest, v2 is the smallest). Then, the triangle is split into
   * two segments. The first segment is the top "half" that includes
   * y-coordinates in the range [v1.y, v0.y]. The second segment is the bottom
   * "half" that includes y-coordinates in the range [v2.y, v1.y]. For each
   * segment, we take horizontal slices and draw lines connecting the left and
   * right boundaries.
   */

  // Sort vertices based on y-coordinates (descending)
  if (v0.y < v1.y)
    std::swap(v0, v1);
  if (v0.y < v2.y)
    std::swap(v0, v2);
  if (v1.y < v2.y)
    std::swap(v1, v2);

  int total_height = v0.y - v2.y;
  for (int y = v0.y; y >= v2.y; y--) {
    bool top_half = (y >= v1.y);
    int segment_height = top_half ? v0.y - v1.y : v1.y - v2.y;

    Vec2i A;
    Vec2i B;

    if (segment_height == 0) {
      A = top_half ? v0 : v1;
      B = top_half ? v1 : v2;
    } else {

      // Alpha and beta are scalars that indicate the progress along a segment
      // as a percentage. These will allow us compute our x coodinates later on.
      float alpha = (float)(v0.y - y) / total_height;
      float beta = (float)((top_half ? v0.y : v1.y) - y) / segment_height;

      // Compute our boundary points.
      //
      // We are travelling across the segments of a triangle. Take our starting
      // vertex (depends on whether we're in the top or bottom "half") and
      // offset it by the vector connecting the two segments. Make sure to
      // multiply the vector by the scalars we computed earlier (indicating how
      // far along we are along a segment).
      A = v0 + (v2 - v0) * alpha;
      B = (top_half ? v0 + (v1 - v0) * beta : v1 + (v2 - v1) * beta);

      // Make sure these are in ascending order so the loop below works
      // correctly.
      if (A.x > B.x)
        std::swap(A, B);
    }

    for (int x = A.x; x <= B.x; x++) {
      image.set(x, y, color);
    }
  }
}

int main(int argc, char **argv) {
  TGAImage image(width, height, TGAImage::RGB);

  // Model *model = new Model("obj/african_head.obj");
  // for (int i = 0; i < model->nfaces(); i++) {
  //   std::vector<int> face = model->face(i);
  //   // Connect all vertices
  //   for (int j = 0; j < 3; j++) {
  //     Vec3f v0 = model->vert(face[j]);
  //     Vec3f v1 = model->vert(face[(j + 1) % 3]);
  //
  //     // x and y values are in the range (-1, 1)
  //     // Want a value of (0, 0) to correspond to the center of our image
  //     // (e.g. for an 800x800 image (0, 0) points from the vertices should
  //     // correspond to (400, 400) in the image).
  //     float x0 = (v0.x + 1) * width / 2.0;
  //     float y0 = (v0.y + 1) * height / 2.0;
  //     float x1 = (v1.x + 1) * width / 2.0;
  //     float y1 = (v1.y + 1) * height / 2.0;
  //
  //     line(x0, y0, x1, y1, image, white);
  //   }
  // }

  Vec2i t0[3] = {Vec2i(10, 70), Vec2i(50, 160), Vec2i(70, 80)};
  Vec2i t1[3] = {Vec2i(180, 50), Vec2i(150, 1), Vec2i(70, 180)};
  Vec2i t2[3] = {Vec2i(180, 150), Vec2i(120, 160), Vec2i(130, 180)};
  Vec2i t4[3] = {Vec2i(180, 150), Vec2i(120, 150), Vec2i(140, 100)};
  Vec2i t5[3] = {Vec2i(180, 0), Vec2i(120, 0), Vec2i(140, 50)};

  triangle(t0[0], t0[1], t0[2], image, red);
  triangle(t1[0], t1[1], t1[2], image, white);
  triangle(t2[0], t2[1], t2[2], image, green);
  triangle(t4[0], t4[1], t4[2], image, green);
  triangle(t5[0], t5[1], t5[2], image, green);

  image.flip_vertically(); // i want to have the origin at the left bottom
                           // corner of the image
  image.write_tga_file("output.tga");
  // delete model;
  return 0;
}
