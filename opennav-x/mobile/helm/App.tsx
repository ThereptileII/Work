import React, {useEffect, useRef, useState} from 'react';
import {
  Button,
  SafeAreaView,
  StyleSheet,
  Text,
  TextInput,
  View,
} from 'react-native';
import NetInfo from '@react-native-community/netinfo';
import * as Keychain from 'react-native-keychain';
import {
  RTCPeerConnection,
  RTCSessionDescription,
  RTCView,
  MediaStream,
} from 'react-native-webrtc';

type ConnectionState = 'offline' | 'idle' | 'pairing' | 'connecting' | 'live' | 'error';
type Signal = {type: 'offer' | 'answer'; sdp?: string};

const SERVICE = 'com.skager.helm.session';

export default function App() {
  const [host, setHost] = useState('http://192.168.0.2:43872');
  const [code, setCode] = useState('');
  const [name, setName] = useState('SKAGER Helm');
  const [state, setState] = useState<ConnectionState>('idle');
  const [message, setMessage] = useState('Status-only autopilot display');
  const [stream, setStream] = useState<MediaStream | undefined>(undefined);
  const pc = useRef<RTCPeerConnection | undefined>(undefined);
  const socket = useRef<WebSocket | undefined>(undefined);
  const dataChannel = useRef<any>(undefined);
  const sequence = useRef(0);
  const [viewerSize, setViewerSize] = useState({width: 1, height: 1});

  useEffect(() => NetInfo.addEventListener(s => {
    if (!s.isConnected) setState('offline');
  }), []);

  const connect = async () => {
    setState('pairing');
    try {
      const response = await fetch(`${host}/helm/v1/pair`, {
        method: 'POST', headers: {'content-type': 'application/json'},
        body: JSON.stringify({pairing_code: code, client_name: name}),
      });
      if (!response.ok) throw new Error('Pairing refused');
      const credentials: {session_id: string; token: string} = await response.json();
      await Keychain.setGenericPassword(credentials.session_id, credentials.token, {service: SERVICE});
      await startPeer(credentials.token);
    } catch (error) {
      setState('error');
      setMessage(error instanceof Error ? error.message : 'Connection failed');
    }
  };

  const startPeer = async (token: string) => {
    setState('connecting');
    const peer = new RTCPeerConnection({iceServers: []});
    pc.current = peer;
    peer.ontrack = (event: any) => {
      if (event.streams[0]) setStream(event.streams[0]);
    };
    peer.onconnectionstatechange = () => {
      const connection = peer.connectionState;
      if (connection === 'connected') { setState('live'); setMessage('Connected over local WebRTC'); }
      if (connection === 'disconnected' || connection === 'failed' || connection === 'closed') {
        setState('error'); setMessage('Helm disconnected; no input is queued');
      }
    };
    const input = peer.createDataChannel('input', {ordered: false, maxRetransmits: 0});
    dataChannel.current = input;
    const ws = new WebSocket(`${host.replace(/^http/, 'ws')}/helm/v1/signal?token=${encodeURIComponent(token)}`);
    socket.current = ws;
    ws.onmessage = async (event: MessageEvent) => {
      const signal = JSON.parse(event.data) as Signal;
      if (signal.type === 'answer' && signal.sdp) await peer.setRemoteDescription(new RTCSessionDescription({type: 'answer', sdp: signal.sdp}));
    };
    ws.onerror = () => { setState('error'); setMessage('Signalling failed'); };
    const offer = await peer.createOffer({offerToReceiveVideo: true});
    await peer.setLocalDescription(offer);
    await new Promise<void>(resolve => {
      if (peer.iceGatheringState === 'complete') return resolve();
      const timer = setInterval(() => {
        if (peer.iceGatheringState === 'complete') { clearInterval(timer); resolve(); }
      }, 50);
    });
    ws.onopen = () => ws.send(JSON.stringify({type: 'offer', sdp: peer.localDescription?.sdp}));
    input.onmessage = (event: any) => setMessage(`Input acknowledgement: ${event.data}`);
  };

  const sendPointer = (x: number, y: number, type: string) => {
    const channel = dataChannel.current;
    if (!channel || channel.readyState !== 'open') return;
    sequence.current += 1;
    channel.send(JSON.stringify({type: 'input', event: {
      type, x: Math.max(0, Math.min(1, x)), y: Math.max(0, Math.min(1, y)),
      sequence: sequence.current, pointer_id: 0,
    }}));
  };

  return <SafeAreaView style={styles.safe}>
    {stream ? <View style={styles.viewer} onLayout={e => setViewerSize({width: e.nativeEvent.layout.width, height: e.nativeEvent.layout.height})} onTouchStart={e => {
      const t = e.nativeEvent.touches[0];
      sendPointer(t.locationX / viewerSize.width, t.locationY / viewerSize.height, 'pointer_down');
    }}><RTCView streamURL={stream.toURL()} style={styles.video} objectFit="contain" /></View> : <View style={styles.form}>
      <Text style={styles.title}>SKAGER Helm</Text>
      <Text style={styles.subtitle}>Local WebRTC companion</Text>
      <TextInput style={styles.input} value={host} onChangeText={setHost} autoCapitalize="none" placeholder="SKAGER PC address" placeholderTextColor="#81929b" />
      <TextInput style={styles.input} value={code} onChangeText={setCode} keyboardType="number-pad" placeholder="Pairing code" placeholderTextColor="#81929b" />
      <TextInput style={styles.input} value={name} onChangeText={setName} placeholder="Device name" placeholderTextColor="#81929b" />
      <Button title="Pair with SKAGER PC" onPress={connect} disabled={state === 'pairing' || state === 'connecting'} />
      <Text style={styles.status}>{state}: {message}</Text>
    </View>}
  </SafeAreaView>;
}

const styles = StyleSheet.create({
  safe: {flex: 1, backgroundColor: '#0d1012'},
  form: {flex: 1, justifyContent: 'center', gap: 14, padding: 24},
  title: {fontSize: 32, fontWeight: '700', color: '#f5f7f8'},
  subtitle: {fontSize: 17, color: '#9caeb8'},
  input: {minHeight: 48, borderWidth: 1, borderColor: '#56636b', borderRadius: 10, padding: 12, color: '#f5f7f8'},
  status: {color: '#9caeb8', marginTop: 8},
  viewer: {flex: 1, backgroundColor: '#000'},
  video: {flex: 1, width: '100%', height: '100%'},
});
