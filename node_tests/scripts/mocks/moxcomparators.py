import mox
import bcolors

class fieldIs(mox.Comparator):
  """Comparison class used to check identity, instead of equality."""

  def __init__(self, field , obj):
    self._obj = obj
    self._field = field

  def equals(self, rhs ):
    return getattr(rhs , self._field) is self._obj

  def __repr__(self):
    return "< field %s is %r >" %  ( self._field ,  self._obj)
    
class fieldEquals(mox.Comparator):
  """Comparison class used to check identity, instead of equality."""

  def __init__(self, field , obj):
    self._obj = obj
    self._field = field

  def equals(self, rhs ):
    return getattr(rhs , self._field) == self._obj

  def __repr__(self):
    return "< field %s is %r >" %  ( self._field ,  self._obj)
    
class fieldGoalEquals(mox.Comparator):
  """Comparison class used to check identity, instead of equality."""

  def __init__(self, field , obj):
    self._obj = obj
    self._field = field

  def equals(self, rhs ):
    return getattr(rhs.goal , self._field) == self._obj

  def __repr__(self):
    return "< field %s is %r >" %  ( self._field ,  self._obj)
    
class Equals(mox.Comparator):
  """Comparison class used to check identity, instead of equality."""

  def __init__(self, obj):
    self._obj = obj

  def equals(self, rhs):
    return rhs == self._obj

  def __repr__(self):
    return "<is %r (%s)>" % (self._obj, id(self._obj))


    
class fieldIsAlmost(mox.Comparator):
  def __init__(self, field , float_value, places=7):
    self._float_value = float_value
    self._places = places
    self._field = field

  def equals(self, rhs):
    rhs = getattr(rhs , self._field)
    self.error =  self._float_value - rhs  
    try:
      return round(rhs-self._float_value, self._places) == 0
    except Exception:
      return False

  def __repr__(self):
    return "< field %s is almost %r , error = %r >" %  ( self._field ,  self._float_value,self.error)
    

class listAnd(mox.Comparator):
  
  def __init__(self, args):
    self._comparators = args

  def equals(self, rhs):
    for comparator in self._comparators:
      if not comparator.equals(rhs):
        return False

    return True

  def __repr__(self):
    return bcolors.bColors.HEADER + '<AND %s>' % str(self._comparators) + bcolors.bColors.ENDC
