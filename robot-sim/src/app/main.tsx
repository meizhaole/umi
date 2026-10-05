import { createRoot } from 'react-dom/client';
import { App } from './App';
import { UmiGripperPreview } from '../viz/UmiGripperPreview';
import './styles.css';

const rootElement = document.getElementById('root');
if (!rootElement) throw new Error('页面缺少 root 挂载节点');

const isUmiGripperPreview =
  new URLSearchParams(window.location.search).get('umi-gripper-preview') === '1';

createRoot(rootElement).render(isUmiGripperPreview ? <UmiGripperPreview /> : <App />);
