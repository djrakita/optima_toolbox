use crate::pair_group_queries::{OPairGroupQryTrait, OParryFilterOutputCategory, OParryPairSelector, ToParryProximityOutputCategory};
use crate::shapes::ShapeCategoryOParryShape;

pub trait AliasParryGroupFilterQry: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=OParryPairSelector, OutputCategory=OParryFilterOutputCategory> { }
impl<A> AliasParryGroupFilterQry for A where A: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=OParryPairSelector, OutputCategory=OParryFilterOutputCategory> { }

pub trait AliasParryToProximityQry: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=OParryPairSelector, OutputCategory=ToParryProximityOutputCategory> { }
impl<A> AliasParryToProximityQry for A where A: OPairGroupQryTrait<ShapeCategory=ShapeCategoryOParryShape, SelectorType=OParryPairSelector, OutputCategory=ToParryProximityOutputCategory> { }