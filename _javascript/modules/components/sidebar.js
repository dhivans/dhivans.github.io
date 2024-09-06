/**
 * Expand or close the sidebar in mobile screens.
 */
############################################################

const sidebar = document.getElementById('sidebar');

sidebar.addEventListener('mouseover', () => {
  sidebar.classList.remove('sidebar-condensed');
});

sidebar.addEventListener('mouseleave', () => {
  sidebar.classList.add('sidebar-condensed');
});

##########################################################


const ATTR_DISPLAY = 'sidebar-display';

class SidebarUtil {
  static isExpanded = false;

  static toggle() {
    if (SidebarUtil.isExpanded === false) {
      document.body.setAttribute(ATTR_DISPLAY, '');
    } else {
      document.body.removeAttribute(ATTR_DISPLAY);
    }

    SidebarUtil.isExpanded = !SidebarUtil.isExpanded;
  }
}

export function sidebarExpand() {
  document
    .getElementById('sidebar-trigger')
    .addEventListener('click', SidebarUtil.toggle);

  document.getElementById('mask').addEventListener('click', SidebarUtil.toggle);
}
