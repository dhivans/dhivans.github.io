module Jekyll
  class ItemCategoryPage < Page
    def initialize(site, base, dir, category)
      @site = site
      @base = base
      @dir = dir
      @name = 'index.html'

      # Set the layout to 'category.html' and pass the category data
      self.process(@name)
      self.read_yaml(File.join(base, '_layouts'), 'category.html')
      self.data['category'] = category
      
      # Set page title explicitly
      self.data['title'] = "#{category}"
      
    end
  end

  class ItemCategoryGenerator < Generator
    safe true

    def generate(site)
      if site.collections.key?('items') && site.posts.docs.any?
        # Gather all post categories
        post_categories = site.posts.docs.flat_map { |post| post.data['categories'] }.uniq

        # Gather item categories and filter them
        item_categories = site.collections['items'].docs.flat_map { |doc| doc.data['categories'] }.uniq
        filtered_categories = item_categories.reject { |category| post_categories.include?(category) }
        
        # Create pages only for item categories not in post categories
        filtered_categories.each do |category|
          site.pages << ItemCategoryPage.new(site, site.source, File.join('categories', category.downcase), category)
        end
      end
    end
  end
end