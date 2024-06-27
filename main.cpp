#include "geometry.h"
#include "model.h"
#include "tgaimage.h"
#include <algorithm>
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

Vec3f barycentric(Vec2i *pts, Vec2i P) {
  /**
   * Computes the barycentric coordinates for a point P with respect to triangle
   * ABC.
   *
   * pts represents the three vertices of the triangle (pts[0] = A, pts[1] = B,
   * pts[2] = C). In a barycentric coordinate system, all points sum to 1 i.e.
   * if we have barycentric coordinates (w, u, v) then w + u + v must sum
   * to 1. We also know that w = 1 - u - v, so we need to find the coordinates:
   * (1 - u - v, u ,v).
   *
   * For a given point P, consider the vector AP (vector from vertex A
   * connecting to point P). We can express the barycentric coordinates for P as
   *
   * P = (1 - u - v) A + u B + v C
   *
   * If we assign weight (1 - u - v) to vertex A, weight u to vertex B, and
   * weight v to vertex C then P must be the center of mass.
   *
   * P = A + u * AB + v * AC
   *
   * Where AB is the vector from A to B and AC is the vector from A to C. Since
   * P is our center of mass, we know:
   *
   * u * AB + v * AC + PA = 0
   *
   * where PA is the vector from P (the point we want to find the barycentric
   * coordinates for) to A.
   *
   * This can be rewritten with matrices as:
   *
   *
   *           [ AB ]
   * [ u v 1 ] [ AC ] = 0
   *           [ PA ]
   *
   * The vector (u, v, 1) must be orthogonal to both the x and y components of
   * the vector (AB, AC, PA). We can find an orthogonal vector by taking the
   * cross product of the x and y components.
   *
   * [ u v 1 ] = (AB_x AC_x PA_x) x (AB_y AC_y PA_y)
   *
   * See here:
   * https://en.wikipedia.org/wiki/Barycentric_coordinate_system#Conversion_between_barycentric_and_Cartesian_coordinates
   */

  // Given a triangle's vertices: A, B, and C. Let pts[0] refer to 1, pts[1]
  // refer to B, and pts[2] refer to C.
  Vec3f orthogonal = Vec3f(pts[2].raw[0] - pts[0].raw[0], // vector AB_x
                           pts[1].raw[0] - pts[0].raw[0], // vector AC_x
                           pts[0].raw[0] - P.raw[0]       // vector PA_x
                           ) ^
                     Vec3f(pts[2].raw[1] - pts[0].raw[1], // vector AB_y
                           pts[1].raw[1] - pts[0].raw[1], // vector AC_y
                           pts[0].raw[1] - P.raw[1]       // vector PA_y
                     );

  // We have the orthogonal vector (u, v, 1) and we know:
  //
  // P = (1 - u - v) A + u B + v C
  //
  // Rearrange the orthogonal vector to get P. Divide by the A component to
  // normalize (want to sum to 1). Note the A component is represented by "z" in
  // the code.

  return Vec3f((orthogonal.z - orthogonal.x - orthogonal.y) / orthogonal.z,
               orthogonal.y / orthogonal.z, orthogonal.x / orthogonal.z);
}

void triangle(Vec2i *pts, TGAImage &image, TGAColor color) {
  // Find a bounding box for the triangle. Then, iterate through points inside
  // the bounding box and determine if they are inside the triangle. We know a
  // point is outside of the triangle if any of its barycentric coordinates are
  // < 0.

  Vec2i bboxmin(image.get_width() - 1, image.get_height() - 1);
  Vec2i bboxmax(0, 0);
  Vec2i clamp(image.get_width() - 1, image.get_height() - 1);

  // Iterate over the triangle's vertices
  for (int i = 0; i < 3; i++) {
    // Find max but make sure we don't come up with a bounding box outside of
    // our image's boundaries (no coordinates larger than (image.width,
    // image.height)).
    bboxmax.x = std::min(clamp.x, std::max(pts[i].x, bboxmax.x));
    bboxmax.y = std::min(clamp.y, std::max(pts[i].y, bboxmax.y));

    // Find min but make sure we don't come up with a bounding box outside of
    // our image's boundaries (no coordinates less than (0, 0)).
    bboxmin.x = std::max(0, std::min(pts[i].x, bboxmin.x));
    bboxmin.y = std::max(0, std::min(pts[i].y, bboxmin.y));
  }

  Vec2i P;
  for (P.x = bboxmin.x; P.x <= bboxmax.x; P.x++) {
    for (P.y = bboxmin.y; P.y <= bboxmax.y; P.y++) {
      Vec3f barycentric_coordinates = barycentric(pts, P);
      if (barycentric_coordinates.x < 0 || barycentric_coordinates.y < 0 ||
          barycentric_coordinates.z < 0) {
        continue;
      }

      image.set(P.x, P.y, color);
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

  triangle(t0, image, red);
  triangle(t1, image, white);
  triangle(t2, image, green);
  triangle(t4, image, green);
  triangle(t5, image, green);

  image.flip_vertically(); // i want to have the origin at the left bottom
                           // corner of the image
  image.write_tga_file("output.tga");
  // delete model;
  return 0;
}
