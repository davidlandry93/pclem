
namespace pclem {
    class BoundingBox {
    public:
        BoundingBox();
        BoundingBox(double xmin, double ymin, double zmin,
                    double xmax, double ymax, double zmax);

        void setMin(double xmin, double ymin, double zmin);
        void setMax(double xmax, double ymax, double zmax);
    private:
        double xmin, ymin, zmin;
        double xmax, ymax, zmax;
    };
}
