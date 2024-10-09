import { basic, initSidebar, initTopbar } from './modules/layouts';
import { initLocaleDatetime, loadImg, categoryCollapse } from './modules/plugins';

loadImg();
initLocaleDatetime();
initSidebar();
initTopbar();
basic();
categoryCollapse();

