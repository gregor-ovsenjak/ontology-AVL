'''import owlready2 as owl


#owl.onto_path.append("file:///C:/Users/grego/Documents/")

onto = owl.get_ontology('file://C:/Users/grego/Documents/mapOntology.owl').load()


Road = onto.Road("Roader")
Roadster = onto.Road("Roader2")


for i in onto.Road.instances(): print(i)

'''

import matplotlib.pyplot as plt


xponts = [35.135869,35.135909,35.135925,35.136053]
yponts = [136.963694,136.963814,136.963533,136.963608]

fig,ax = plt.subplots()

ax.scatter(xponts,yponts)


plt.show()