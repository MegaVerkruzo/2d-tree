#include "primitives.h"

#include <algorithm>
#include <charconv>
#include <fstream>
#include <map>

std::ostream & operator<<(std::ostream & out, const Point & point)
{
    return out << point.x() << " " << point.y();
}

double Rect::distance(const Point & p) const
{
    if (contains(p)) {
        return 0;
    }
    else if (is_sorted_numbers(xmin(), p.x(), xmax())) {
        return std::min(std::abs(ymax() - p.y()), std::abs(y_min - p.y()));
    }
    else if (is_sorted_numbers(ymin(), p.y(), ymax())) {
        return std::min(std::abs(xmax() - p.x()), std::abs(xmin() - p.x()));
    }
    return std::min(
            std::min(p.distance(Point(xmin(), ymin())),
                     p.distance(Point(xmin(), ymax()))),
            std::min(p.distance(Point(xmax(), ymin())),
                     p.distance(Point(xmax(), ymax()))));
}

bool Rect::intersects(const Rect & rect) const
{
    return any_corner_in_rect(rect) || rect.any_corner_in_rect(*this) ||
            (xmin() <= rect.xmin() && rect.xmax() <= xmax() && rect.ymin() <= ymin() && ymax() <= rect.ymax()) ||
            (ymin() <= rect.ymin() && rect.ymax() <= ymax() && rect.xmin() <= xmin() && xmax() <= rect.xmax());
}

std::ostream & operator<<(std::ostream & out, const rbtree::PointSet & set)
{
    for (const auto item : set) {
        out << item.x() << ' ' << item.y() << '\n';
    }
    return out;
}

rbtree::PointSet::PointSet(const std::string & input)
{
    std::ifstream in(input);
    if (!in.is_open() && !input.empty()) {
        throw "Unable to access this file";
    }
    double x, y;
    while (in >> x >> y) {
        m_set.insert(Point(x, y));
    }
}

std::pair<rbtree::PointSet::iterator, rbtree::PointSet::iterator> rbtree::PointSet::range(const Rect & rect) const
{
    std::set<Point> result;
    for (const auto & elem : m_set) {
        if (rect.contains(elem)) {
            result.insert(elem);
        }
    }
    auto ans = std::make_shared<std::set<Point>>(result);
    return {iterator(ans->begin(), ans), iterator(ans->end(), ans)};
}

std::optional<Point> rbtree::PointSet::nearest(const Point & point) const
{
    if (m_set.empty()) {
        return {};
    }

    return *std::min_element(m_set.begin(), m_set.end(), [&point](const Point & left, const Point & right) {
        return point.distance(left) < point.distance(right);
    });
}

std::pair<rbtree::PointSet::iterator, rbtree::PointSet::iterator> rbtree::PointSet::nearest(const Point & point, std::size_t k) const
{
    std::map<double, Point, std::greater<double>> distances;
    for (iterator current = begin(); current != end(); ++current) {
        if (distances.size() < k || (k > 0 && point.distance(*current) < point.distance(distances.begin()->second))) {
            distances.insert({point.distance(*current), *current});
        }
        if (distances.size() > k) {
            distances.erase(distances.begin());
        }
    }
    std::set<Point> result;
    for (const auto & elem : distances) {
        result.insert(elem.second);
    }
    auto ans = std::make_shared<std::set<Point>>(std::move(result));
    return {iterator(ans->begin(), ans), iterator(ans->end(), ans)};
}

std::shared_ptr<kdtree::PointSet::Node> kdtree::PointSet::put(const std::shared_ptr<kdtree::PointSet::Node> & current, const Point & point, bool x_value)
{
    if (current == nullptr) {
        return std::make_shared<Node>(Node(point, x_value));
    }
    if (point == current->point) {
        return current;
    }
    if ((point < current->point && current->x_value) || (point.compare_y(current->point) && !current->x_value)) {
        current->left = put(current->left, point, !x_value);
    }
    else {
        current->right = put(current->right, point, !x_value);
    }
    update_node(current);
    return current;
}

void kdtree::PointSet::update_node(const std::shared_ptr<kdtree::PointSet::Node> & current)
{
    if (current != nullptr) {
        current->size = size(current->left) + size(current->right) + 1;
        update_parent(current->left, current);
        update_parent(current->right, current);
    }
}

bool kdtree::PointSet::contains(const std::shared_ptr<kdtree::PointSet::Node> & current, const Point & point) const
{
    if (current == nullptr) {
        return false;
    }
    if (current->point == point) {
        return true;
    }
    else if ((point < current->point && current->x_value) || (point.compare_y(current->point) && !current->x_value)) {
        return contains(current->left, point);
    }
    else {
        return contains(current->right, point);
    }
}

kdtree::PointSet::PointSet(const std::string & filename)
{
    std::ifstream in(filename);
    if (!in.is_open() && !filename.empty()) {
        throw "Unable to access this file";
    }
    double x, y;
    while (in >> x >> y) {
        put(Point(x, y));
    }
}

std::shared_ptr<kdtree::PointSet::Node> kdtree::PointSet::left(const std::shared_ptr<kdtree::PointSet::Node> & current)
{
    if (current == nullptr || current->left == nullptr) {
        return current;
    }
    return left(current->left);
}

std::shared_ptr<kdtree::PointSet::Node> kdtree::PointSet::next_node(std::shared_ptr<kdtree::PointSet::Node> current)
{
    if (current->right != nullptr) {
        return left(current->right);
    }
    std::weak_ptr<Node> next = current->parent;
    while (!next.expired() && current == next.lock()->right) {
        current = next.lock();
        next = next.lock()->parent;
    }
    return next.lock();
}

std::ostream & operator<<(std::ostream & out, const kdtree::PointSet & pointSet)
{
    for (kdtree::PointSet::iterator iter = pointSet.begin(); iter != pointSet.end(); ++iter) {
        out << *iter;
    }
    return out;
}

std::pair<kdtree::PointSet::iterator, kdtree::PointSet::iterator> kdtree::PointSet::range(const Rect & rect) const
{
    auto point_set = std::make_shared<PointSet>(PointSet());
    range(point_set, m_root, rect, Rect(Point(-std::numeric_limits<double>::max(), -std::numeric_limits<double>::max()), Point(std::numeric_limits<double>::max(), std::numeric_limits<double>::max())));
    return {iterator(point_set->left(point_set->m_root), point_set), iterator(nullptr, point_set)};
}

void kdtree::PointSet::range(std::shared_ptr<PointSet> & point_set, const std::shared_ptr<kdtree::PointSet::Node> & current, const Rect & rect, const Rect & border) const
{
    if (current == nullptr) {
        return;
    }
    if (rect.contains(current->point)) {
        point_set->put(current->point);
    }
    auto check_child = [&point_set, &current, &rect, &border, this](const bool left) {
        Rect new_border = make_intersection(border, current->point, current->x_value, left);
        if (new_border.xmin() <= new_border.xmax() && rect.intersects(new_border)) {
            range(point_set, (left ? current->left : current->right), rect, new_border);
        }
    };
    check_child(true);
    check_child(false);
}

Rect kdtree::PointSet::make_intersection(const Rect & border, const Point & border_point, bool x_value, bool left) const
{
    Point bottom_left_point(
            (x_value && !left ? std::max(border.xmin(), border_point.x()) : border.xmin()),
            (!x_value && !left ? std::max(border_point.y(), border.ymin()) : border.ymin()));
    Point top_right_point(
            (x_value && left ? std::min(border.xmax(), border_point.x()) : border.xmax()),
            (!x_value && left ? std::min(border.ymax(), border_point.y()) : border.ymax()));
    return Rect(bottom_left_point, top_right_point);
}
std::optional<Point> kdtree::PointSet::nearest(const Point & point) const
{
    if (empty()) {
        return {};
    }
    return nearest(m_root, m_root, point)->point;
}

std::pair<kdtree::PointSet::iterator, kdtree::PointSet::iterator> kdtree::PointSet::nearest(const Point & point, std::size_t k) const
{
    std::map<double, Point, std::greater<double>> distances;
    for (iterator current = begin(); current != end(); ++current) {
        if (distances.size() < k || (k > 0 && point.distance(*current) < point.distance(distances.begin()->second))) {
            distances.insert({point.distance(*current), *current});
        }
        if (distances.size() > k) {
            distances.erase(distances.begin());
        }
    }
    auto result = std::make_shared<PointSet>(PointSet());
    for (const auto & elem : distances) {
        result->put(elem.second);
    }
    return std::make_pair(iterator(result->left(result->m_root), result), iterator(nullptr, result));
}

kdtree::PointSet::PointSet(const std::pair<iterator, iterator> & interval)
{
    const auto & [begining, ending] = interval;
    for (auto iter = begining; iter != ending; ++iter) {
        put(*iter);
    }
}

kdtree::PointSet::PointSet(const kdtree::PointSet & pointSet)
{
    for (auto iter = pointSet.begin(); iter != pointSet.end(); ++iter) {
        put(*iter);
    }
}

std::shared_ptr<kdtree::PointSet::Node> kdtree::PointSet::nearest(const std::shared_ptr<Node> & node, std::shared_ptr<Node> nearest_node, const Point & p) const
{
    if (node == nullptr) {
        return nearest_node;
    }
    double p_coordinate = (node->x_value ? p.x() : p.y());
    auto first_result = (p_coordinate < node->point.x() ? node->left : node->right);
    auto second_result = (p_coordinate < node->point.x() ? node->right : node->left);
    nearest_node = nearest(first_result, nearest_node, p);
    if (p.distance(nearest_node->point) > p.distance(node->point)) {
        nearest_node = node;
    }
    double nearest_node_coordinate = (node->x_value ? nearest_node->point.x() : nearest_node->point.y());
    if (p.distance(nearest_node->point) > std::abs(p_coordinate - nearest_node_coordinate)) {
        nearest_node = nearest(second_result, nearest_node, p);
    }
    return nearest_node;
}
