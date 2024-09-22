module Jekyll
    class ItemCategoryPage < Page
      def initialize(site, base, dir, category)
        @site = site
        @base = base
        @dir = dir
        @name = 'index.html'
  
        self.process(@name)
        self.read_yaml(File.join(base, '_layouts'), 'category-items.html')
        self.data['category'] = category
        self.data['title'] = "Category: #{category.capitalize}"
      end
    end
  
    class ItemCategoryGenerator < Generator
      safe true
  
      def generate(site)
        if site.collections.key?('items')
          categories = site.collections['items'].docs.flat_map { |doc| doc.data['categories'] }.uniq
          
          categories.each do |category|
            site.pages << ItemCategoryPage.new(site, site.source, File.join('categories/items', category.downcase), category)
          end
        end
      end
    end
  end
  