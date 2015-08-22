
class Lego():
    """ A Lego object, which is the object to be manipulated in the experiment. """

    def __init__(self, idnum, location, color_upper, color_lower, descriptor_words):
        """
        Location is a length 3 array of floats. Color is a length 3 array of
        ints represeting an RGB array for upper and lower threshold values. 
        Descriptor words is an array of words.
        """
        self.idnum = idnum
        self.loc = location
        self.color_upper = color_upper
        self.color_lower = color_lower
        self.words = descriptor_words
