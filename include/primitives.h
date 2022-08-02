#pragma once

#include <cmath>
#include <iostream>
#include <memory>
#include <optional>
#include <ostream>
#include <set>

namespace {
double square(const double & x)
{
    return x * x;
}

bool is_sorted_numbers(double left, double middle, double right)
{
    return left <= middle && middle <= right;
}
} // namespace

class Point
{
public:
    Point(double x, double y)
        : x_coordinate(x)
        , y_coordinate(y)
    {
    }

    double x() const
    {
        return x_coordinate;
    }

    double y() const
    {
        return y_coordinate;
    }

    double distance(const Point & point) const
    {
        return std::sqrt(square(point.x() - x()) + square(point.y() - y()));
    }

    bool operator<(const Point & point) const
    {
        return x() < point.x() || (x() == point.x() && y() < point.y());
    }
    bool operator>(const Point & point) const
    {
        return point < *this;
    }

    bool operator<=(const Point & point) const
    {
        return *this < point || *this == point;
    }

    bool operator>=(const Point & point) const
    {
        return point <= *this;
    }

    bool operator==(const Point & point) const
    {
        return std::abs(x() - point.x()) < std::numeric_limits<double>::min() && std::abs(y() - point.y()) < std::numeric_limits<double>::min();
    }

    bool operator!=(const Point & point) const
    {
        return !(*this == point);
    }

    bool compare_y(const Point & point) const
    {
        return y() < point.y() || (y() == point.y() && x() <= point.x());
    }

private:
    double x_coordinate;
    double y_coordinate;
};

class Rect
{
public:
    Rect(const Point & left_bottom, const Point & right_top)
        : x_min(left_bottom.x())
        , x_max(right_top.x())
        , y_min(left_bottom.y())
        , y_max(right_top.y())
    {
    }

    double xmin() const
    {
        return x_min;
    }

    double ymin() const
    {
        return y_min;
    }

    double xmax() const
    {
        return x_max;
    }

    double ymax() const
    {
        return y_max;
    }

    double distance(const Point &) const;

    bool contains(const Point & p) const
    {
        return is_sorted_numbers(xmin(), p.x(), xmax()) && is_sorted_numbers(ymin(), p.y(), ymax());
    }

    bool intersects(const Rect &) const;

    bool any_corner_in_rect(const Rect & rect) const
    {
        return rect.contains(Point(xmin(), ymin())) || rect.contains(Point(xmin(), ymax())) || rect.contains(Point(xmax(), ymin())) || rect.contains(Point(xmax(), ymax()));
    }

private:
    double x_min;
    double x_max;
    double y_min;
    double y_max;
};

namespace rbtree {

class PointSet
{
public:
    class iterator
    {
    public:
        using difference_type = std::ptrdiff_t;
        using value_type = Point;
        using pointer = const value_type *;
        using reference = const value_type &;
        using iterator_category = std::forward_iterator_tag;

        iterator(const std::set<Point>::iterator current,
                 std::shared_ptr<std::set<Point>> data = nullptr)
            : m_current(current)
            , m_data(std::move(data))
        {
        }

        iterator() = default;

        reference operator*() const
        {
            return *m_current;
        }

        pointer operator->() const
        {
            return &*(m_current);
        }

        iterator & operator++()
        {
            ++m_current;
            return *this;
        }

        auto operator++(int)
        {
            iterator result = *this;
            operator++();
            return result;
        }

        friend bool operator==(const iterator & left, const iterator & right)
        {
            return left.m_current == right.m_current;
        }

        friend bool operator!=(const iterator & left, const iterator & right)
        {
            return !(left == right);
        }

    private:
        std::set<Point>::iterator m_current;
        std::shared_ptr<std::set<Point>> m_data;
    };

    PointSet(const std::string & input = {});

    PointSet(const std::pair<iterator, iterator> & input_set)
    {
        m_set.insert(input_set.first, input_set.second);
    }

    bool empty() const
    {
        return m_set.empty();
    }

    std::size_t size() const
    {
        return m_set.size();
    }

    void put(const Point & point)
    {
        m_set.insert(point);
    }

    bool contains(const Point & point) const
    {
        return m_set.find(point) != m_set.end();
    }

    //         second iterator points to an element out of range
    std::pair<iterator, iterator> range(const Rect &) const;
    iterator begin() const
    {
        return {m_set.begin()};
    }

    iterator end() const
    {
        return {m_set.end()};
    }

    std::optional<Point> nearest(const Point &) const;
    //    // second iterator points to an element out of range
    std::pair<iterator, iterator> nearest(const Point & p, std::size_t k) const;

    friend std::ostream & operator<<(std::ostream &, const PointSet &);

private:
    std::set<Point> m_set;
};

} // namespace rbtree

namespace kdtree {

class PointSet
{
public:
    struct Node
    {
        Point point;
        bool x_value : 1;
        size_t size = 1;
        std::shared_ptr<Node> left = nullptr;
        std::shared_ptr<Node> right = nullptr;
        std::weak_ptr<Node> parent;

        Node(Point current_point, bool current_x_value, const std::shared_ptr<Node> & current_parent = nullptr)
            : point(current_point)
            , x_value(current_x_value)
            , parent(current_parent)
        {
        }
    };

    class iterator
    {
    public:
        using difference_type = std::ptrdiff_t;
        using value_type = Point;
        using pointer = const value_type *;
        using reference = const value_type &;
        using iterator_category = std::forward_iterator_tag;

        iterator(const std::shared_ptr<Node> & node, const std::shared_ptr<PointSet> & pointSet)
            : m_current(node)
            , m_point_set(pointSet)
        {
        }

        iterator(const std::shared_ptr<Node> & node)
            : m_current(node)
        {
        }

        iterator() = default;

        reference operator*() const
        {
            return m_current->point;
        }

        pointer operator->() const
        {
            return &m_current->point;
        }

        iterator & operator++()
        {
            m_current = next_node(m_current);
            return *this;
        }

        auto operator++(int)
        {
            iterator result = *this;
            operator++();
            return result;
        }

        friend bool operator==(const iterator & left, const iterator & right)
        {
            return left.m_point_set == right.m_point_set && left.m_current == right.m_current;
        }

        friend bool operator!=(const iterator & left, const iterator & right)
        {
            return !(left == right);
        }

    private:
        friend class PointSet;

        std::shared_ptr<Node> m_current = nullptr;
        std::shared_ptr<PointSet> m_point_set = nullptr;
    };

    PointSet(const std::string & filename = {});

    PointSet(const std::pair<iterator, iterator> & iter);

    PointSet(const PointSet & pointSet);

    std::size_t size() const
    {
        return size(m_root);
    }

    bool empty() const
    {
        return size() == 0;
    }

    bool contains(const Point & point) const
    {
        return contains(m_root, point);
    }

    void put(const Point & point)
    {
        m_root = put(m_root, point, true);
        update_node(m_root);
    }

    std::pair<iterator, iterator> range(const Rect &) const;

    iterator begin() const
    {
        return iterator(left(m_root));
    }

    iterator end() const
    {
        return iterator(nullptr);
    }

    std::optional<Point> nearest(const Point &) const;
    std::pair<iterator, iterator> nearest(const Point &, std::size_t) const;

    friend std::ostream & operator<<(std::ostream &, const PointSet &);

    static std::shared_ptr<Node> left(const std::shared_ptr<Node> & current);

    static std::shared_ptr<Node> next_node(std::shared_ptr<Node> current);

private:
    std::shared_ptr<Node> m_root;

    void update_node(const std::shared_ptr<Node> & current);

    void update_parent(const std::shared_ptr<Node> & current, const std::shared_ptr<Node> & parent)
    {
        if (current != nullptr) {
            current->parent = parent;
        }
    }

    std::size_t size(const std::shared_ptr<Node> & current) const
    {
        if (current == nullptr) {
            return 0;
        }
        return current->size;
    }

    std::shared_ptr<Node> put(const std::shared_ptr<Node> &, const Point &, bool);

    bool contains(const std::shared_ptr<Node> &, const Point &) const;

    void range(std::shared_ptr<PointSet> & point_set, const std::shared_ptr<Node> &, const Rect & rect, const Rect & border) const;

    Rect make_intersection(const Rect & border, const Point & border_point, bool x_value, bool left) const;

    std::shared_ptr<Node> nearest(const std::shared_ptr<Node> & node, std::shared_ptr<Node> nearest_node, const Point & p) const;
};

} // namespace kdtree
