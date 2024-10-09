#include <mimik/vision/google_cloud_ocr.h>

namespace mimik {
namespace vision {

std::vector<google::cloud::vision::v1::BoundingPoly>
getDocumentBounds(const google::cloud::vision::v1::TextAnnotation& document, FeatureType feature)
{
  std::vector<google::cloud::vision::v1::BoundingPoly> bounds;

  for (auto const& page : document.pages())
  {
    for (auto const& block : page.blocks())
    {
      for (auto const& paragraph : block.paragraphs())
      {
        for (auto const& word : paragraph.words())
        {
          for (auto const& symbol : word.symbols())
          {
            if (feature == FeatureType::SYMBOL)
              bounds.push_back(symbol.bounding_box());
          }
          if (feature == FeatureType::WORD)
            bounds.push_back(word.bounding_box());
        }
        if (feature == FeatureType::PARA)
          bounds.push_back(paragraph.bounding_box());
      }
      if (feature == FeatureType::BLOCK)
        bounds.push_back(block.bounding_box());
    }
  }
  return bounds;
}

}  // namespace vision
}  // namespace mimik
