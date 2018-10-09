#ifndef DIMENSION_H_
#define DIMENSION_H_

class Dimension {
 public:
  static Dimension fromWidthAndHeight(double width, double height);
  double getWidth() const;
  double getHeight() const;

 private:
  Dimension(double width, double height);
  double width;
  double height;
};

Dimension::Dimension(double _width, double _height)
    : width(_width),
      height(_height) {
}

Dimension Dimension::fromWidthAndHeight(double width, double height) {
  return Dimension(width, height);
}

double Dimension::getWidth() const {
  return width;
}

double Dimension::getHeight() const {
  return height;
}

#endif /* DIMENSION_H_ */
