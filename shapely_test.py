import shapely
from shapely import Polygon, MultiPolygon
from shapely.validation import make_valid as make_val
poly1 = Polygon([
                    (0,0),(0,1),(1,1),(1,0)
                ])
poly2 = Polygon([
                    (0.1,0.1),(0.1,0.2),(0.2,0.2),(0.2,0.1)
                ])
ob = MultiPolygon([poly1, poly2])

print('IS VALID: ',ob.is_valid)

print('using make_valid()')
shapely.make_valid(ob)


print(ob)
print(type(ob))

print('---- MAKE VALID ----')

polygon1 = Polygon([(0,0), (1,1), (1,2), (1,1), (0,0)])
polygon2 = Polygon([(0,0),(0,1),(1,1),(1,0),(0,0)])


print('Polygon1: ', polygon1)
print('Polygon1 IS_VALID: ', polygon1.is_valid)

print('Polygon2: ', polygon2)
print('Polygon2 IS_VALID: ', polygon2.is_valid)

new_polygon1 = shapely.make_valid(polygon1)
print('New_polygon1: ', new_polygon1)
print('New_Polygon1 IS_VALID: ', new_polygon1.is_valid)
print()
print()
new_polygon2 = make_val(polygon2)
print('New_polygon2: ', new_polygon2)
print('New_Polygon2 IS_VALID: ', new_polygon2.is_valid)

print()
print()
multipolygon = MultiPolygon([polygon1, polygon2])


print('Multipolygon: ', multipolygon)
print('Multipolygon IS_VALID: ', multipolygon.is_valid)

new_multipolygon = make_val(multipolygon)
print('New_multipolygon: ', new_multipolygon)
print(type(new_multipolygon))
print()
print('Geometries: ') 
# use a for loop - to loop through the geometries in accessible through geoms(attr)
# check - IF polygon - add to list of geometries that will be used for airspace
# as an aside - see how different are the two lists before and after removal 
print(new_multipolygon.geoms[0])
print()
print('New_multipolygon IS_VALID: ', new_multipolygon)
print()
print('Printing individual geoms: ')
for geom in new_multipolygon.geoms:
    if isinstance(geom, Polygon):
        print(geom)
    else:
        print('Not a polygon: ',geom)


# remove all non-polygon out of the geometry collection list
