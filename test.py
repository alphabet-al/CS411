

test_set = set()

test_set.add(((4,5), 'west', 1))
test_set.add(((5,4), 'south', 1))
test_set.add(((3,5), 'west', 1))

node = (3,5)

exists = any(node in t for t in test_set)

print(exists)