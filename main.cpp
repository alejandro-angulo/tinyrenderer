#include "geometry.h"
#include "model.h"
#include "tgaimage.h"
#include <algorithm>
#include <iostream>
#include <limits>
#include <utility>

const TGAColor white = TGAColor(255, 255, 255, 255);
const TGAColor red = TGAColor(255, 0, 0, 255);
const TGAColor green = TGAColor(0, 255, 0, 255);
const TGAColor blue = TGAColor(0, 0, 255, 255);
const int height = 800;
const int width = 800;
const Vec3f light_direction(0, 0, -1); // Where the light is coming from.

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

Vec3f barycentric(Vec3f A, Vec3f B, Vec3f C, Vec3f P) {
  /**
   * Computes the barycentric coordinates for a point P with respect to triangle
   * ABC.
   *
   * In a barycentric coordinate system, all points sum to 1 i.e.
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

  Vec3f s[2]; // Will hold our AB, AC, and PA vectors (x- and y-components)
  for (int i = 0; i < 2; i++) {
    s[i][0] = C[i] - A[i];
    s[i][1] = B[i] - A[i];
    s[i][2] = A[i] - P[i];
  }

  Vec3f orthogonal = cross(s[0], s[1]);

  // We have the orthogonal vector (u, v, 1) and we know:
  //
  // P = (1 - u - v) A + u B + v C
  //
  // Rearrange the orthogonal vector to get P. Divide by the A component to
  // normalize (want to sum to 1). Note the A component is represented by "z" in
  // the code.

  // if (std::abs(orthogonal.z) < 1e-2) {
  //   return Vec3f(-1, 1, 1);
  // }

  return Vec3f(1.0f - (orthogonal.x + orthogonal.y) / orthogonal.z,
               orthogonal.y / orthogonal.z, orthogonal.x / orthogonal.z);
}

void triangle(Vec3f *pts, float *zbuffer, TGAImage &image, TGAColor color) {
  Vec2f bboxmin(std::numeric_limits<float>::max(),
                std::numeric_limits<float>::max());
  Vec2f bboxmax(-std::numeric_limits<float>::max(),
                -std::numeric_limits<float>::max());
  Vec2f clamp(image.get_width() - 1, image.get_height() - 1);

  // Iterate over the triangle's vertices
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 2; j++) {
      // Find max but make sure we don't come up with a bounding box outside of
      // our image's boundaries (no coordinates larger than (image.width,
      // image.height)).
      bboxmax[j] = std::min(clamp[j], std::max(bboxmax[j], pts[i][j]));

      // Find min but make sure we don't come up with a bounding box outside of
      // our image's boundaries (no coordinates less than (0, 0)).
      bboxmin[j] = std::max(0.f, std::min(bboxmin[j], pts[i][j]));
    }
  }

  Vec3f P;
  for (P.x = bboxmin.x; P.x <= bboxmax.x; P.x++) {
    for (P.y = bboxmin.y; P.y <= bboxmax.y; P.y++) {
      Vec3f bc_screen = barycentric(pts[0], pts[1], pts[2], P);
      if (bc_screen.x < 0 || bc_screen.y < 0 || bc_screen.z < 0)
        continue;
      P.z = 0;
      for (int i = 0; i < 3; i++)
        P.z += pts[i][2] * bc_screen[i];
      if (zbuffer[int(P.x + P.y * width)] < P.z) {
        zbuffer[int(P.x + P.y * width)] = P.z;
        image.set(P.x, P.y, color);
      }
    }
  }
}

int main(int argc, char **argv) {
  TGAImage image(width, height, TGAImage::RGB);

  Model *model = new Model("obj/african_head.obj");
  float zbuffer[width * height];

  // XXX: For some reason this doesn't work right (ends up rendering a
  // "skinnier" model)
  // std::fill_n(zbuffer, width * height, -std::numeric_limits<float>::min());

  for (int i = width * height; i--;
       zbuffer[i] = -std::numeric_limits<float>::max())
    ;

  for (int i = 0; i < model->nfaces(); i++) {
    std::vector<int> face = model->face(i);
    Vec3f screen_coords[3];
    Vec3f world_coords[3];
    for (int j = 0; j < 3; j++) {
      // Convert world coordinates (what's in our 3d model) to coordinates on
      // our screen. Take the center of our screen (halfway across and
      // halfway up) to be the origin .
      world_coords[j] = model->vert(face[j]);

      // Add 1 to our world coordinates since conversion of a float to an int
      // truncates our value (e.g. a float of 2.3 becomes an int of 2).
      screen_coords[j] =
          Vec3f(int((world_coords[j].x + 1.0) * width / 2.0 + 0.5),
                int((world_coords[j].y + 1.0) * height / 2.0 + 0.5),
                world_coords[j].z);
    }

    // Want to get a vector perpendicular to our triangle
    Vec3f normal = cross((world_coords[2] - world_coords[0]) // Vector XZ
                         ,
                         (world_coords[1] - world_coords[0]) // vector XY
                         )
                       .normalize();
    // Light intesity is the dot product of our normal vector and our light's
    // direction. This means that the intensity is highest when our face is
    // perpendicular to the light (we take the dot product of the normal
    // which is already perpendicular to our face and dot product is largest
    // when parallel).
    float intensity = normal * light_direction;

    // Don't bother drawing a triangle if intensity <= 0 .
    // An intensity of 0 means there's no light on it all (completely dark).
    // A negative intensity means that the light hits from behind (back
    // culling -- skipping triangles that shouldn't be visible).
    if (intensity > 0) {
      triangle(screen_coords, zbuffer, image,
               TGAColor(intensity * 255, intensity * 255, intensity * 255, 255)
               // TGAColor(rand() % 255, rand() % 255, rand() % 255, 255)

      );
    }
  }

  image.flip_vertically(); // i want to have the origin at the left bottom
                           // corner of the image
  image.write_tga_file("output.tga");
  delete model;
  return 0;
}
