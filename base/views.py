from django.shortcuts import render

from base.path_finder import PathFinder


def home(request):
    finder = PathFinder('input_data/points.xml', 'input_data/polygons.xml')
    print(finder.points)
    print(finder.polygons)
    #finder.calculateVisibiltyGraph()
    #finder.dijkstra()
    return render(request, 'home.html')
